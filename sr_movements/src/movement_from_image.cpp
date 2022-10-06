/*
* Copyright 2019, 2022 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation version 2 of the License.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program. If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * @file   movement_from_image.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Tue Sep 27 10:05:01 2011
 *
 * @brief  Reads a png file and creates a movement from it.
 *
 *
 */

#include <iostream>
#include <string>

#include "sr_movements/movement_from_image.hpp"
#include <ros/ros.h>

namespace shadowrobot
{
  MovementFromImage::MovementFromImage(std::string image_path)
    : PartialMovement()
  {
    image_ = boost::shared_ptr<Magick::Image>( new Magick::Image() );
    image_->read(image_path);

    nb_cols_ = image_->columns();
    nb_rows_ = image_->rows();

    generate_movement_();
  }

  MovementFromImage::~MovementFromImage()
  {}


  void MovementFromImage::generate_movement_()
  {
    Magick::Color white("white");

    for (ssize_t col = 0; col < nb_cols_; ++col)
    {
      bool no_pixel = true;
      for (ssize_t row = 0; row < nb_rows_; ++row)
      {
        if (image_->pixelColor(row, col) != white)
        {
          no_pixel = false;
          steps.push_back(1.0 - static_cast<double>(row) / static_cast<double>(nb_rows_));
          break;
        }
      }
      if (no_pixel)
      {
        // not sending any targets for this point.
        steps.push_back(-1.0);
      }
    }
  }
}  // namespace shadowrobot

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
