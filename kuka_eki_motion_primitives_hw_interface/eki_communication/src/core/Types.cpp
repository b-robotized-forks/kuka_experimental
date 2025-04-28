#include <eki_communication/core/Types.h>

#include <math.h>

rbt::GripPositions::GripPositions(float item_size, float force_distance)
{
    item_size *= 100 / 2;
    force_distance *= 100;

    float min_position = 75;   // 1.5 mm
    float max_position = 7575; // 151.5 mm

    float target_position = max_position - item_size + 4750;

    base = min_position;

    shift = std::fmax(target_position - force_distance / 2, base + 1);
    teach = std::fmax(target_position, shift + 1);
    work = std::fmax(target_position + force_distance / 2, teach + 1);

    work = std::fmin(work, max_position);
    teach = std::fmin(teach, work - 1);
    shift = std::fmin(shift, teach - 1);
}
