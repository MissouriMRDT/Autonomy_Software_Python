# Rover State Machine

|                        | Idle       | Navigating    | SearchPattern     | ApproachingMarker |
| ---------------------- | ---------- | ------------- | ----------------- | ----------------- |
| START                  | Navigating | Navigating    | SearchPattern     | -                 |
| REACHED_GPS_COORDINATE | -          | SearchPattern | -                 | -                 |
| MARKER_SIGHTED         | -          | -             | ApproachingMarker | -                 |
| SEARCH_FAILED          | -          | -             | Navigating        | -                 |
| MARKER_UNSEEN          | -          | -             | -                 | SearchPattern     |
| REACHED_MARKER         | -          | -             | -                 | Idle              |
| ALL_MARKERS_REACHED    | -          | -             | -                 | -                 |
| ABORT                  | Idle       | Idle          | Idle              | Idle              |
| RESTART                | -          | -             | -                 | -                 |
| OBSTACLE_AVOIDANCE     | -          | -             | -                 | -                 |
| END_OBSTACLE_AVOIDANCE | -          | -             | -                 | -                 |
| NO_WAYPOINT            | -          | Idle          | -                 | -                 |
| NEW_WAYPOINT           | -          | Navigating    | -                 | -                 |
