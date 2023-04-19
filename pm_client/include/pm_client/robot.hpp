#pragma once

#include <memory>

#include "open62541/open62541.h"

#include "pm_client/aerotech_axis.hpp"

namespace PMClient
{

class Robot
{
  private:
    int status = 0;

  public:
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
        return true;
    }
};

} // namespace PMClient
