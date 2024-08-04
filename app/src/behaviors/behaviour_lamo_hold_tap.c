#define DT_DRV_COMPAT zmk_behavior_lamo_hold_tap

#include <zephyr/device.h>
#include <drivers/behavior.h>
#include <zmk/keys.h>
#include <dt-bindings/zmk/keys.h>
#include <zephyr/logging/log.h>
#include <zmk/behavior.h>
#include <zmk/matrix.h>
#include <zmk/endpoints.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/behavior.h>
#include <zmk/keymap.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

//#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#define ZMK_BHV_lamo_hold_tap_MAX_HELD 10
#define ZMK_BHV_lamo_hold_tap_MAX_CAPTURED_EVENTS 40

// increase if you have keyboard with more keys.
#define ZMK_BHV_lamo_hold_tap_POSITION_NOT_USED 9999

enum status {
    STATUS_UNDECIDED,
    STATUS_TAP,
    STATUS_HOLD_INTERRUPT,
    STATUS_HOLD_TIMER,
};

enum decision_moment {
    HT_KEY_DOWN,
    HT_KEY_UP,
    HT_OTHER_KEY_DOWN,
    HT_OTHER_KEY_UP,
    HT_TIMER_EVENT,
    HT_QUICK_TAP,
};

struct behavior_lamo_hold_tap_config {
    int tapping_term_ms;
    char *hold_behavior_dev;
    char *tap_behavior_dev;
    int quick_tap_ms;
    int require_prior_idle_ms;
    bool hold_while_undecided;
    bool hold_while_undecided_linger;
    bool retro_tap;
    bool hold_trigger_on_release;
    int32_t hold_trigger_key_positions_len;
    int32_t hold_trigger_key_positions[];
};

// this data is specific for each hold-tap
struct active_lamo_hold_tap {
    int32_t position;
    uint32_t param_hold;
    uint32_t param_tap;
    int64_t timestamp;
    enum status status;
    const struct behavior_lamo_hold_tap_config *config;
    struct k_work_delayable work;
    bool work_is_cancelled;

    // initialized to -1, which is to be interpreted as "no other key has been pressed yet"
    int32_t position_of_first_other_key_pressed;
    int32_t position_of_first_other_key_released;
};

// The undecided hold tap is the hold tap that needs to be decided before
// other keypress events can be released. While the undecided_lamo_hold_tap is
// not NULL, most events are captured in captured_events.
// After the lamo_hold_tap is decided, it will stay in the active_lamo_hold_taps until
// its key-up has been processed and the delayed work is cleaned up.
struct active_lamo_hold_tap *undecided_lamo_hold_tap = NULL;
struct active_lamo_hold_tap active_lamo_hold_taps[ZMK_BHV_lamo_hold_tap_MAX_HELD] = {};
// We capture most position_state_changed events and some modifiers_state_changed events.

enum captured_event_tag {
    ET_NONE,
    ET_POS_CHANGED,
    ET_CODE_CHANGED,
};

union captured_event_data {
    struct zmk_position_state_changed_event position;
    struct zmk_keycode_state_changed_event keycode;
};

struct captured_event {
    enum captured_event_tag tag;
    union captured_event_data data;
};

struct captured_event captured_events[ZMK_BHV_lamo_hold_tap_MAX_CAPTURED_EVENTS] = {};

// Keep track of which key was tapped most recently for the standard, if it is a hold-tap
// a position, will be given, if not it will just be INT32_MIN
struct last_tapped {
    int32_t position;
    int64_t timestamp;
};

// Set time stamp to large negative number initially for test suites, but not
// int64 min since it will overflow if -1 is added
struct last_tapped last_tapped = {INT32_MIN, INT32_MIN};

static void store_last_tapped(int64_t timestamp) {
    if (timestamp > last_tapped.timestamp) {
        last_tapped.position = INT32_MIN;
        last_tapped.timestamp = timestamp;
    }
}

static void store_last_lamo_hold_tapped(struct active_lamo_hold_tap *lamo_hold_tap) {
    last_tapped.position = lamo_hold_tap->position;
    last_tapped.timestamp = lamo_hold_tap->timestamp;
}

static bool is_quick_tap(struct active_lamo_hold_tap *lamo_hold_tap) {
    if ((last_tapped.timestamp + lamo_hold_tap->config->require_prior_idle_ms) > lamo_hold_tap->timestamp) {
        return true;
    } else {
        return (last_tapped.position == lamo_hold_tap->position) &&
               (last_tapped.timestamp + lamo_hold_tap->config->quick_tap_ms) > lamo_hold_tap->timestamp;
    }
}

static int capture_event(struct captured_event *data) {
    for (int i = 0; i < ZMK_BHV_lamo_hold_tap_MAX_CAPTURED_EVENTS; i++) {
        if (captured_events[i].tag == ET_NONE) {
            captured_events[i] = *data;
            return 0;
        }
    }
    return -ENOMEM;
}

static bool have_captured_keydown_event(uint32_t position) {
    for (int i = 0; i < ZMK_BHV_lamo_hold_tap_MAX_CAPTURED_EVENTS; i++) {
        struct captured_event *ev = &captured_events[i];
        if (ev->tag == ET_NONE) {
            return false;
        }

        if (ev->tag != ET_POS_CHANGED) {
            continue;
        }

        if (ev->data.position.data.position == position && ev->data.position.data.state) {
            return true;
        }
    }
    return false;
}

const struct zmk_listener zmk_listener_behavior_lamo_hold_tap;

static void release_captured_events() {
    if (undecided_lamo_hold_tap != NULL) {
        return;
    }

    // We use a trick to prevent copying the captured_events array.
    //
    // Events for different mod-tap instances are separated by a NULL pointer.
    //
    // The first event popped will never be catched by the next active hold-tap
    // because to start capturing a mod-tap-key-down event must first completely
    // go through the events queue.
    //
    // Example of this release process;
    // [mt2_down, k1_down, k1_up, mt2_up, null, ...]
    //  ^
    // mt2_down position event isn't captured because no hold-tap is active.
    // mt2_down behavior event is handled, now we have an undecided hold-tap
    // [null, k1_down, k1_up, mt2_up, null, ...]
    //        ^
    // k1_down  is captured by the mt2 mod-tap
    // !note that searches for find_captured_keydown_event by the mt2 behavior will stop at the
    // first null encountered [mt1_down, null, k1_up, mt2_up, null, ...]
    //                  ^
    // k1_up event is captured by the new hold-tap:
    // [k1_down, k1_up, null, mt2_up, null, ...]
    //                        ^
    // mt2_up event is not captured but causes release of mt2 behavior
    // [k1_down, k1_up, null, null, null, ...]
    // now mt2 will start releasing it's own captured positions.
    for (int i = 0; i < ZMK_BHV_lamo_hold_tap_MAX_CAPTURED_EVENTS; i++) {
        struct captured_event *captured_event = &captured_events[i];
        enum captured_event_tag tag = captured_event->tag;

        if (tag == ET_NONE) {
            return;
        }

        captured_events[i].tag = ET_NONE;
        if (undecided_lamo_hold_tap != NULL) {
            k_msleep(10);
        }

        switch (tag) {
        case ET_CODE_CHANGED:
            LOG_DBG("Releasing mods changed event 0x%02X %s",
                    captured_event->data.keycode.data.keycode,
                    (captured_event->data.keycode.data.state ? "pressed" : "released"));
            ZMK_EVENT_RAISE_AT(captured_event->data.keycode, behavior_lamo_hold_tap);
            break;
        case ET_POS_CHANGED:
            LOG_DBG("Releasing key position event for position %d %s",
                    captured_event->data.position.data.position,
                    (captured_event->data.position.data.state ? "pressed" : "released"));
            ZMK_EVENT_RAISE_AT(captured_event->data.position, behavior_lamo_hold_tap);
            break;
        default:
            LOG_ERR("Unhandled captured event type");
            break;
        }
    }
}

static struct active_lamo_hold_tap *find_lamo_hold_tap(uint32_t position) {
    for (int i = 0; i < ZMK_BHV_lamo_hold_tap_MAX_HELD; i++) {
        if (active_lamo_hold_taps[i].position == position) {
            return &active_lamo_hold_taps[i];
        }
    }
    return NULL;
}

static struct active_lamo_hold_tap *store_lamo_hold_tap(uint32_t position, uint32_t param_hold,
                                              uint32_t param_tap, int64_t timestamp,
                                              const struct behavior_lamo_hold_tap_config *config) {
    for (int i = 0; i < ZMK_BHV_lamo_hold_tap_MAX_HELD; i++) {
        if (active_lamo_hold_taps[i].position != ZMK_BHV_lamo_hold_tap_POSITION_NOT_USED) {
            continue;
        }
        active_lamo_hold_taps[i].position = position;
        active_lamo_hold_taps[i].status = STATUS_UNDECIDED;
        active_lamo_hold_taps[i].config = config;
        active_lamo_hold_taps[i].param_hold = param_hold;
        active_lamo_hold_taps[i].param_tap = param_tap;
        active_lamo_hold_taps[i].timestamp = timestamp;
        active_lamo_hold_taps[i].position_of_first_other_key_pressed = -1;
        active_lamo_hold_taps[i].position_of_first_other_key_released = -1;
        return &active_lamo_hold_taps[i];
    }
    return NULL;
}

static void clear_lamo_hold_tap(struct active_lamo_hold_tap *lamo_hold_tap) {
    lamo_hold_tap->position = ZMK_BHV_lamo_hold_tap_POSITION_NOT_USED;
    lamo_hold_tap->status = STATUS_UNDECIDED;
    lamo_hold_tap->work_is_cancelled = false;
}

static void decide_balanced(struct active_lamo_hold_tap *lamo_hold_tap, enum decision_moment event) {
    switch (event) {
    case HT_KEY_UP:
        lamo_hold_tap->status = STATUS_TAP;
        return;
    case HT_OTHER_KEY_UP:
        lamo_hold_tap->status = STATUS_HOLD_INTERRUPT;
        return;
    case HT_TIMER_EVENT:
        lamo_hold_tap->status = STATUS_HOLD_TIMER;
        return;
    case HT_QUICK_TAP:
        lamo_hold_tap->status = STATUS_TAP;
        return;
    default:
        return;
    }
}

static void decide_tap_preferred(struct active_lamo_hold_tap *lamo_hold_tap, enum decision_moment event) {
    switch (event) {
    case HT_KEY_UP:
        lamo_hold_tap->status = STATUS_TAP;
        return;
    case HT_TIMER_EVENT:
        lamo_hold_tap->status = STATUS_HOLD_TIMER;
        return;
    case HT_QUICK_TAP:
        lamo_hold_tap->status = STATUS_TAP;
        return;
    default:
        return;
    }
}

static void decide_tap_unless_interrupted(struct active_lamo_hold_tap *lamo_hold_tap,
                                          enum decision_moment event) {
    switch (event) {
    case HT_KEY_UP:
        lamo_hold_tap->status = STATUS_TAP;
        return;
    case HT_OTHER_KEY_DOWN:
        lamo_hold_tap->status = STATUS_HOLD_INTERRUPT;
        return;
    case HT_TIMER_EVENT:
        lamo_hold_tap->status = STATUS_TAP;
        return;
    case HT_QUICK_TAP:
        lamo_hold_tap->status = STATUS_TAP;
        return;
    default:
        return;
    }
}

static void decide_hold_preferred(struct active_lamo_hold_tap *lamo_hold_tap, enum decision_moment event) {
    switch (event) {
    case HT_KEY_UP:
        lamo_hold_tap->status = STATUS_TAP;
        return;
    case HT_OTHER_KEY_DOWN:
        lamo_hold_tap->status = STATUS_HOLD_INTERRUPT;
        return;
    case HT_TIMER_EVENT:
        lamo_hold_tap->status = STATUS_HOLD_TIMER;
        return;
    case HT_QUICK_TAP:
        lamo_hold_tap->status = STATUS_TAP;
        return;
    default:
        return;
    }
}

static inline const char *status_str(enum status status) {
    switch (status) {
    case STATUS_UNDECIDED:
        return "undecided";
    case STATUS_HOLD_TIMER:
        return "hold-timer";
    case STATUS_HOLD_INTERRUPT:
        return "hold-interrupt";
    case STATUS_TAP:
        return "tap";
    default:
        return "UNKNOWN STATUS";
    }
}

static inline const char *decision_moment_str(enum decision_moment decision_moment) {
    switch (decision_moment) {
    case HT_KEY_UP:
        return "key-up";
    case HT_OTHER_KEY_DOWN:
        return "other-key-down";
    case HT_OTHER_KEY_UP:
        return "other-key-up";
    case HT_QUICK_TAP:
        return "quick-tap";
    case HT_TIMER_EVENT:
        return "timer";
    default:
        return "UNKNOWN STATUS";
    }
}

static int press_hold_binding(struct active_lamo_hold_tap *lamo_hold_tap) {
    struct zmk_behavior_binding_event event = {
        .position = lamo_hold_tap->position,
        .timestamp = lamo_hold_tap->timestamp,
    };

    struct zmk_behavior_binding binding = {.behavior_dev = lamo_hold_tap->config->hold_behavior_dev,
                                           .param1 = lamo_hold_tap->param_hold};
    return behavior_keymap_binding_pressed(&binding, event);
}

static int press_tap_binding(struct active_lamo_hold_tap *lamo_hold_tap) {
    struct zmk_behavior_binding_event event = {
        .position = lamo_hold_tap->position,
        .timestamp = lamo_hold_tap->timestamp,
    };

    struct zmk_behavior_binding binding = {.behavior_dev = lamo_hold_tap->config->tap_behavior_dev,
                                           .param1 = lamo_hold_tap->param_tap};
    store_last_lamo_hold_tapped(lamo_hold_tap);
    return behavior_keymap_binding_pressed(&binding, event);
}

static int release_hold_binding(struct active_lamo_hold_tap *lamo_hold_tap) {
    struct zmk_behavior_binding_event event = {
        .position = lamo_hold_tap->position,
        .timestamp = lamo_hold_tap->timestamp,
    };

    struct zmk_behavior_binding binding = {.behavior_dev = lamo_hold_tap->config->hold_behavior_dev,
                                           .param1 = lamo_hold_tap->param_hold};
    return behavior_keymap_binding_released(&binding, event);
}

static int release_tap_binding(struct active_lamo_hold_tap *lamo_hold_tap) {
    struct zmk_behavior_binding_event event = {
        .position = lamo_hold_tap->position,
        .timestamp = lamo_hold_tap->timestamp,
    };

    struct zmk_behavior_binding binding = {.behavior_dev = lamo_hold_tap->config->tap_behavior_dev,
                                           .param1 = lamo_hold_tap->param_tap};
    return behavior_keymap_binding_released(&binding, event);
}

static int press_binding(struct active_lamo_hold_tap *lamo_hold_tap) {
    if (lamo_hold_tap->config->retro_tap && lamo_hold_tap->status == STATUS_HOLD_TIMER) {
        return 0;
    }

    if (lamo_hold_tap->status == STATUS_HOLD_TIMER || lamo_hold_tap->status == STATUS_HOLD_INTERRUPT) {
        if (lamo_hold_tap->config->hold_while_undecided) {
            // the hold is already active, so we don't need to press it again
            return 0;
        } else {
            return press_hold_binding(lamo_hold_tap);
        }
    } else {
        if (lamo_hold_tap->config->hold_while_undecided &&
            !lamo_hold_tap->config->hold_while_undecided_linger) {
            // time to release the hold before pressing the tap
            release_hold_binding(lamo_hold_tap);
        }
        return press_tap_binding(lamo_hold_tap);
    }
}

static int release_binding(struct active_lamo_hold_tap *lamo_hold_tap) {
    if (lamo_hold_tap->config->retro_tap && lamo_hold_tap->status == STATUS_HOLD_TIMER) {
        return 0;
    }

    if (lamo_hold_tap->status == STATUS_HOLD_TIMER || lamo_hold_tap->status == STATUS_HOLD_INTERRUPT) {
        return release_hold_binding(lamo_hold_tap);
    } else {
        return release_tap_binding(lamo_hold_tap);
    }
}

static bool is_first_other_key_pressed_trigger_key(struct active_lamo_hold_tap *lamo_hold_tap) {
    for (int i = 0; i < lamo_hold_tap->config->hold_trigger_key_positions_len; i++) {
        if (lamo_hold_tap->config->hold_trigger_key_positions[i] ==
            lamo_hold_tap->position_of_first_other_key_pressed) {
            return true;
        }
    }
    return false;
}

static bool is_first_other_key_released_trigger_key(struct active_lamo_hold_tap *lamo_hold_tap) {
    for (int i = 0; i < lamo_hold_tap->config->hold_trigger_key_positions_len; i++) {
        if (lamo_hold_tap->config->hold_trigger_key_positions[i] ==
            lamo_hold_tap->position_of_first_other_key_released) {
            return true;
        }
    }
    return false;
}

static void decide_positional_hold(struct active_lamo_hold_tap *lamo_hold_tap) {

    // Positional conditions is not active?
    if (!(lamo_hold_tap->config->hold_trigger_key_positions_len > 0)) {
        return; // apply flavour
    }

    // Pressed key is not set?
    if (lamo_hold_tap->position_of_first_other_key_pressed == -1) {
        return; // apply flavor
    }

    // Pressed key is included in positions?
    if (is_first_other_key_pressed_trigger_key(lamo_hold_tap)) {
        return; // apply flavor
    }

    // Pressed key is not included in positions.
    // We act on press?
    if (undecided_lamo_hold_tap->config->hold_trigger_on_release == false) {
        lamo_hold_tap->status = STATUS_TAP;
        return; // ignore flavor, set TAP
    }

    // We act on release.
    // Released key is not set?
    if (lamo_hold_tap->position_of_first_other_key_released == -1) {
        // Is current decision hold based on key pressed?
        if (lamo_hold_tap->status == STATUS_HOLD_INTERRUPT) {

            // We can't decide yet if key which will be released:
            // - not in positions
            // - be released before timer
            // So we can't decide yet if we should overwrite decision to TAP.
            // We have to wait for key release.

            lamo_hold_tap->status = STATUS_UNDECIDED;
            return; // remove flavor
        }

        // There decision is decision:
        // - STATUS_HOLD_TIMER - tapping term reached, apply flavor
        // - STATUS_TAP - even if we set TAP later it will not change decision
        return; // apply flavor
    }

    // Released key is included in positions?
    if (is_first_other_key_released_trigger_key(lamo_hold_tap)) {
        return; // apply flavor
    }

    // Released key is not included in positions.
    lamo_hold_tap->status = STATUS_TAP;
    return; // ignore flavor, set TAP
}

static void decide_lamo_hold_tap(struct active_lamo_hold_tap *lamo_hold_tap,
                            enum decision_moment decision_moment) { //HT_QUICKTAP
    if (lamo_hold_tap->status != STATUS_UNDECIDED) {
        return;
    }

    if (lamo_hold_tap != undecided_lamo_hold_tap) {
        LOG_DBG("ERROR found undecided tap hold that is not the active tap hold");
        return;
    }

    if (lamo_hold_tap->config->hold_while_undecided && decision_moment == HT_KEY_DOWN) {
        LOG_DBG("%d hold behavior pressed while undecided", lamo_hold_tap->position);
        press_hold_binding(lamo_hold_tap);
        return;
    }

    decide_balanced(lamo_hold_tap, decision_moment); //status -> TAP
    
    if (lamo_hold_tap->status == STATUS_UNDECIDED) {
        return;
    }

    decide_positional_hold(lamo_hold_tap);

    if (lamo_hold_tap->status == STATUS_UNDECIDED) {
        return;
    }

    if (lamo_hold_tap->status == STATUS_TAP && decision_moment == HT_QUICK_TAP) {
        decision_moment = HT_KEY_DOWN;
        decide_tap_preferred(lamo_hold_tap, decision_moment);
    }

    // Since the hold-tap has been decided, clean up undecided_lamo_hold_tap and
    // execute the decided behavior.
    LOG_DBG("%d decided %s (%s decision moment %s)", lamo_hold_tap->position,
            status_str(lamo_hold_tap->status), "balanced",
            decision_moment_str(decision_moment));
    undecided_lamo_hold_tap = NULL;
    press_binding(lamo_hold_tap);
    release_captured_events();
}

static void decide_retro_tap(struct active_lamo_hold_tap *lamo_hold_tap) {
    if (!lamo_hold_tap->config->retro_tap) {
        return;
    }
    if (lamo_hold_tap->status == STATUS_HOLD_TIMER) {
        release_binding(lamo_hold_tap);
        LOG_DBG("%d retro tap", lamo_hold_tap->position);
        lamo_hold_tap->status = STATUS_TAP;
        press_binding(lamo_hold_tap);
        return;
    }
}

static void update_hold_status_for_retro_tap(uint32_t ignore_position) {
    for (int i = 0; i < ZMK_BHV_lamo_hold_tap_MAX_HELD; i++) {
        struct active_lamo_hold_tap *lamo_hold_tap = &active_lamo_hold_taps[i];
        if (lamo_hold_tap->position == ignore_position ||
            lamo_hold_tap->position == ZMK_BHV_lamo_hold_tap_POSITION_NOT_USED ||
            lamo_hold_tap->config->retro_tap == false) {
            continue;
        }
        if (lamo_hold_tap->status == STATUS_HOLD_TIMER) {
            LOG_DBG("Update hold tap %d status to hold-interrupt", lamo_hold_tap->position);
            lamo_hold_tap->status = STATUS_HOLD_INTERRUPT;
            press_binding(lamo_hold_tap);
        }
    }
}

static int on_lamo_hold_tap_binding_pressed(struct zmk_behavior_binding *binding,
                                       struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_lamo_hold_tap_config *cfg = dev->config;

    if (undecided_lamo_hold_tap != NULL) {
        LOG_DBG("ERROR another hold-tap behavior is undecided.");
        // if this happens, make sure the behavior events occur AFTER other position events.
        return ZMK_BEHAVIOR_OPAQUE;
    }

    struct active_lamo_hold_tap *lamo_hold_tap =
        store_lamo_hold_tap(event.position, binding->param1, binding->param2, event.timestamp, cfg);
    if (lamo_hold_tap == NULL) {
        LOG_ERR("unable to store hold-tap info, did you press more than %d hold-taps?",
                ZMK_BHV_lamo_hold_tap_MAX_HELD);
        return ZMK_BEHAVIOR_OPAQUE;
    }

    LOG_DBG("%d new undecided lamo_hold_tap", event.position);
    undecided_lamo_hold_tap = lamo_hold_tap;

    if (is_quick_tap(lamo_hold_tap)) {
        decide_lamo_hold_tap(lamo_hold_tap, HT_QUICK_TAP);
    }

    decide_lamo_hold_tap(lamo_hold_tap, HT_KEY_DOWN);

    // if this behavior was queued we have to adjust the timer to only
    // wait for the remaining time.
    int32_t tapping_term_ms_left = (lamo_hold_tap->timestamp + cfg->tapping_term_ms) - k_uptime_get();
    k_work_schedule(&lamo_hold_tap->work, K_MSEC(tapping_term_ms_left));

    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_lamo_hold_tap_binding_released(struct zmk_behavior_binding *binding,
                                        struct zmk_behavior_binding_event event) {
    struct active_lamo_hold_tap *lamo_hold_tap = find_lamo_hold_tap(event.position);
    if (lamo_hold_tap == NULL) {
        LOG_ERR("ACTIVE_lamo_hold_tap_CLEANED_UP_TOO_EARLY");
        return ZMK_BEHAVIOR_OPAQUE;
    }

    // If these events were queued, the timer event may be queued too late or not at all.
    // We insert a timer event before the TH_KEY_UP event to verify.
    int work_cancel_result = k_work_cancel_delayable(&lamo_hold_tap->work);
    if (event.timestamp > (lamo_hold_tap->timestamp + lamo_hold_tap->config->tapping_term_ms)) {
        decide_lamo_hold_tap(lamo_hold_tap, HT_TIMER_EVENT);
    }

    decide_lamo_hold_tap(lamo_hold_tap, HT_KEY_UP);
    decide_retro_tap(lamo_hold_tap);
    release_binding(lamo_hold_tap);

    if (lamo_hold_tap->config->hold_while_undecided && lamo_hold_tap->config->hold_while_undecided_linger) {
        release_hold_binding(lamo_hold_tap);
    }

    if (work_cancel_result == -EINPROGRESS) {
        // let the timer handler clean up
        // if we'd clear now, the timer may call back for an uninitialized active_lamo_hold_tap.
        LOG_DBG("%d hold-tap timer work in event queue", event.position);
        lamo_hold_tap->work_is_cancelled = true;
    } else {
        LOG_DBG("%d cleaning up hold-tap", event.position);
        clear_lamo_hold_tap(lamo_hold_tap);
    }

    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_lamo_hold_tap_driver_api = {
    .binding_pressed = on_lamo_hold_tap_binding_pressed,
    .binding_released = on_lamo_hold_tap_binding_released,
};

static int position_state_changed_listener(const zmk_event_t *eh) {
    struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);

    update_hold_status_for_retro_tap(ev->position);

    if (undecided_lamo_hold_tap == NULL) {
        LOG_DBG("%d bubble (no undecided lamo_hold_tap active)", ev->position);
        return ZMK_EV_EVENT_BUBBLE;
    }

    // Store the position of pressed key for positional hold-tap purposes.
    if (ev->state && (undecided_lamo_hold_tap->position_of_first_other_key_pressed == -1)) {
        undecided_lamo_hold_tap->position_of_first_other_key_pressed = ev->position;
    }

    // Store the position of pressed key for positional hold-tap purposes.
    if (!ev->state && undecided_lamo_hold_tap->config->hold_trigger_on_release &&
        (undecided_lamo_hold_tap->position_of_first_other_key_released == -1)) {
        undecided_lamo_hold_tap->position_of_first_other_key_released = ev->position;
    }

    if (undecided_lamo_hold_tap->position == ev->position) {
        if (ev->state) { // keydown
            LOG_ERR("hold-tap listener should be called before before most other listeners!");
            return ZMK_EV_EVENT_BUBBLE;
        } else { // keyup
            LOG_DBG("%d bubble undecided hold-tap keyrelease event", undecided_lamo_hold_tap->position);
            return ZMK_EV_EVENT_BUBBLE;
        }
    }

    // If these events were queued, the timer event may be queued too late or not at all.
    // We make a timer decision before the other key events are handled if the timer would
    // have run out.
    if (ev->timestamp >
        (undecided_lamo_hold_tap->timestamp + undecided_lamo_hold_tap->config->tapping_term_ms)) {
        decide_lamo_hold_tap(undecided_lamo_hold_tap, HT_TIMER_EVENT);
    }

    if (undecided_lamo_hold_tap == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    if (!ev->state && !have_captured_keydown_event(ev->position)) {
        // no keydown event has been captured, let it bubble.
        // we'll catch modifiers later in modifier_state_changed_listener
        LOG_DBG("%d bubbling %d %s event", undecided_lamo_hold_tap->position, ev->position,
                ev->state ? "down" : "up");
        return ZMK_EV_EVENT_BUBBLE;
    }

    LOG_DBG("%d capturing %d %s event", undecided_lamo_hold_tap->position, ev->position,
            ev->state ? "down" : "up");
    struct captured_event capture = {
        .tag = ET_POS_CHANGED,
        .data = {.position = copy_raised_zmk_position_state_changed(ev)},
    };
    capture_event(&capture);
    decide_lamo_hold_tap(undecided_lamo_hold_tap, ev->state ? HT_OTHER_KEY_DOWN : HT_OTHER_KEY_UP);
    return ZMK_EV_EVENT_CAPTURED;
}

static int keycode_state_changed_listener(const zmk_event_t *eh) {
    // we want to catch layer-up events too... how?
    struct zmk_keycode_state_changed *ev = as_zmk_keycode_state_changed(eh);

    if (ev->state && !is_mod(ev->usage_page, ev->keycode)) {
        store_last_tapped(ev->timestamp);
    }

    if (undecided_lamo_hold_tap == NULL) {
        // LOG_DBG("0x%02X bubble (no undecided lamo_hold_tap active)", ev->keycode);
        return ZMK_EV_EVENT_BUBBLE;
    }

    if (!is_mod(ev->usage_page, ev->keycode)) {
        // LOG_DBG("0x%02X bubble (not a mod)", ev->keycode);
        return ZMK_EV_EVENT_BUBBLE;
    }

    // hold-while-undecided can produce a mod, but we don't want to capture it.
    if (undecided_lamo_hold_tap->config->hold_while_undecided &&
        undecided_lamo_hold_tap->status == STATUS_UNDECIDED) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    // only key-up events will bubble through position_state_changed_listener
    // if a undecided_lamo_hold_tap is active.
    LOG_DBG("%d capturing 0x%02X %s event", undecided_lamo_hold_tap->position, ev->keycode,
            ev->state ? "down" : "up");
    struct captured_event capture = {
        .tag = ET_CODE_CHANGED, .data = {.keycode = copy_raised_zmk_keycode_state_changed(ev)}};
    capture_event(&capture);
    return ZMK_EV_EVENT_CAPTURED;
}

int behavior_lamo_hold_tap_listener(const zmk_event_t *eh) {
    if (as_zmk_position_state_changed(eh) != NULL) {
        return position_state_changed_listener(eh);
    } else if (as_zmk_keycode_state_changed(eh) != NULL) {
        return keycode_state_changed_listener(eh);
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(behavior_lamo_hold_tap, behavior_lamo_hold_tap_listener);
ZMK_SUBSCRIPTION(behavior_lamo_hold_tap, zmk_position_state_changed);
// this should be modifiers_state_changed, but unfrotunately that's not implemented yet.
ZMK_SUBSCRIPTION(behavior_lamo_hold_tap, zmk_keycode_state_changed);

void behavior_lamo_hold_tap_timer_work_handler(struct k_work *item) {
    struct k_work_delayable *d_work = k_work_delayable_from_work(item);
    struct active_lamo_hold_tap *lamo_hold_tap = CONTAINER_OF(d_work, struct active_lamo_hold_tap, work);

    if (lamo_hold_tap->work_is_cancelled) {
        clear_lamo_hold_tap(lamo_hold_tap);
    } else {
        decide_lamo_hold_tap(lamo_hold_tap, HT_TIMER_EVENT);
    }
}

static int behavior_lamo_hold_tap_init(const struct device *dev) {
    static bool init_first_run = true;

    if (init_first_run) {
        for (int i = 0; i < ZMK_BHV_lamo_hold_tap_MAX_HELD; i++) {
            k_work_init_delayable(&active_lamo_hold_taps[i].work, behavior_lamo_hold_tap_timer_work_handler);
            active_lamo_hold_taps[i].position = ZMK_BHV_lamo_hold_tap_POSITION_NOT_USED;
        }
    }
    init_first_run = false;
    return 0;
}

#define KP_INST(n)                                                                                 \
    static struct behavior_lamo_hold_tap_config behavior_lamo_hold_tap_config_##n = {                        \
        .tapping_term_ms = DT_INST_PROP(n, tapping_term_ms),                                       \
        .hold_behavior_dev = DEVICE_DT_NAME(DT_INST_PHANDLE_BY_IDX(n, bindings, 0)),               \
        .tap_behavior_dev = DEVICE_DT_NAME(DT_INST_PHANDLE_BY_IDX(n, bindings, 1)),                \
        .quick_tap_ms = DT_INST_PROP(n, quick_tap_ms),                                             \
        .require_prior_idle_ms = DT_INST_PROP(n, global_quick_tap)                                 \
                                     ? DT_INST_PROP(n, quick_tap_ms)                               \
                                     : DT_INST_PROP(n, require_prior_idle_ms),                     \
        .hold_while_undecided = DT_INST_PROP(n, hold_while_undecided),                             \
        .hold_while_undecided_linger = DT_INST_PROP(n, hold_while_undecided_linger),               \
        .retro_tap = DT_INST_PROP(n, retro_tap),                                                   \
        .hold_trigger_on_release = DT_INST_PROP(n, hold_trigger_on_release),                       \
        .hold_trigger_key_positions = DT_INST_PROP(n, hold_trigger_key_positions),                 \
        .hold_trigger_key_positions_len = DT_INST_PROP_LEN(n, hold_trigger_key_positions),         \
    };                                                                                             \
    BEHAVIOR_DT_INST_DEFINE(n, behavior_lamo_hold_tap_init, NULL, NULL, &behavior_lamo_hold_tap_config_##n,  \
                            POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                      \
                            &behavior_lamo_hold_tap_driver_api);

DT_INST_FOREACH_STATUS_OKAY(KP_INST)

//#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
