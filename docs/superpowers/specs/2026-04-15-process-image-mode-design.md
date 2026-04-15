# ProcessImage Mode-Aware Vision Design

## Goal

Make `/pointAI/process_image` distinguish between:

1. Adaptive-height vision requests from the cabin driver
2. Bind-execution vision requests from the module driver

so that each path applies the correct Z-gating rule and returns actionable failure details.

## Existing Root Cause

`fast_image_solve/ProcessImage.srv` currently has an empty request body, so the vision node cannot know why the driver is calling it. The current Python implementation therefore applies one shared stable-Z policy to all callers.

## Required Behavior

### Adaptive-height mode

- Driver: `suoquNode.cpp`
- Condition to release points downstream:
  - within 3 frames
  - same point indices
  - each point's world Z remains within `+-5mm`
- No `94mm` ceiling check in this mode

### Bind-check mode

- Driver: `moduanNode.cpp`
- Condition to release points downstream:
  - within 3 frames
  - same point indices
  - each point's world Z remains within `+-5mm`
  - all points satisfy `z <= 94mm`
- If stable points exist but one or more points exceed `94mm`:
  - vision returns failure
  - response message explicitly says the points are not within `94mm`
  - response includes how many points exceeded the limit
  - response includes each violating point index and actual Z
  - module driver logs the failure reason

## Interface Design

Update `fast_image_solve/ProcessImage.srv` to include:

### Request

- `uint8 request_mode`
- constants:
  - `MODE_DEFAULT=0`
  - `MODE_ADAPTIVE_HEIGHT=1`
  - `MODE_BIND_CHECK=2`

`MODE_DEFAULT` should behave like adaptive-height mode for backward compatibility with old callers.

### Response

- `bool success`
- `string message`
- `int32 out_of_height_count`
- `int32[] out_of_height_point_indices`
- `float32[] out_of_height_z_values`
- existing point payload fields remain

## Vision Logic

Once a stable 3-frame window is found:

- adaptive mode:
  - return success with points
- bind-check mode:
  - if all points `<= 94mm`, return success with points
  - else return failure immediately with details

Polling remains inside vision while:

- no valid points are found
- the stable 3-frame requirement is not yet satisfied

## Driver Behavior

### `suoquNode.cpp`

- Set `request_mode = MODE_ADAPTIVE_HEIGHT` before calling `/pointAI/process_image`
- If response is unsuccessful, log the returned message and skip height adaptation for that cycle

### `moduanNode.cpp`

- Set `request_mode = MODE_BIND_CHECK` before calling `/pointAI/process_image`
- If service call fails: keep current failure behavior
- If service call succeeds but `response.success == false`:
  - log returned message with module-driver prefix
  - return service failure to upper layer

## Verification

- Source-level tests for the new `.srv` schema
- Unit tests for mode-specific Z validation helpers in `pointAI.py`
- Rebuild at least `fast_image_solve` and `chassis_ctrl`
