#pragma once

#include <memory>

#include "open62541/open62541.h"

#include "pm_client/aerotech_axis.hpp"
#include "pm_client/camera.hpp"
#include "pm_client/pneumatic_cylinder.hpp"

namespace PMClient
{

class Robot
{
  private:
    int status = 0;

  public:
    // TODO: Array for axes and pneumatics?

    /**
     * X-Axis.
     */
    std::unique_ptr<AerotechAxis> x_axis;

    /**
     * Y-Axis.
     */
    std::unique_ptr<AerotechAxis> y_axis;

    /**
     * Z-Axis.
     */
    std::unique_ptr<AerotechAxis> z_axis;

    /**
     * T-Axis.
     */
    std::unique_ptr<AerotechAxis> t_axis;

    /**
     * Top camera.
     */
    std::unique_ptr<Camera1> camera1;

    /**
     * Bottom camera.
     */
    std::unique_ptr<Camera2> camera2;

    std::unique_ptr<PneumaticCylinder> uv1_pneumatic;

    std::unique_ptr<PneumaticCylinder> uv2_pneumatic;

    std::unique_ptr<PneumaticCylinder> glue_pneumatic;

    std::unique_ptr<PneumaticCylinder> glue_2k_pneumatic;

    std::unique_ptr<PneumaticCylinder> camera_mire_pneumatic;

    /**
     * Check if all axis references are properly set.
     *
     * \returns `true` if yes, otherwise `false`.
     */
    bool is_ok()
    {
        auto axes = {x_axis.get(), y_axis.get(), z_axis.get(), t_axis.get()};
        for (const auto &axis : axes)
        {
            if (axis == nullptr || !axis->is_ok())
            {
                return false;
            }
        }
        auto pneumatics = {
            uv1_pneumatic.get(),
            uv2_pneumatic.get(),
            glue_pneumatic.get(),
            glue_2k_pneumatic.get(),
            camera_mire_pneumatic.get()};
        for (const auto &pneumatic : pneumatics)
        {
            if (pneumatic == nullptr || !pneumatic->is_ok())
            {
                return false;
            }
        }
        return camera1->is_ok() && camera2->is_ok();
    }

    [[nodiscard]] AerotechAxis &get_axis(AxisId id)
    {
        switch (id)
        {
            case AxisId::X:
                return *x_axis;
            case AxisId::Y:
                return *y_axis;
            case AxisId::Z:
                return *z_axis;
            case AxisId::T:
                return *t_axis;
        }
    }

    [[nodiscard]] PneumaticCylinder &get_pneumatic(PneumaticId id)
    {
        switch (id)
        {
            case PneumaticId::UV1:
                return *uv1_pneumatic;
            case PneumaticId::UV2:
                return *uv2_pneumatic;
            case PneumaticId::Glue:
                return *glue_pneumatic;
            case PneumaticId::Glue2K:
                return *glue_2k_pneumatic;
            case PneumaticId::CameraMire:
                return *camera_mire_pneumatic;
        }
    }
};

} // namespace PMClient
