/*
* Copyright 2019 Shadow Robot Company Ltd.
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
 * @file   movement_from_image.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Tue Sep 27 10:05:01 2011
 *
 * @brief  Reads a png file and creates a movement from it.
 *
 *
 */


#ifndef _MOVEMENT_FROM_IMAGE_HPP_
#define _MOVEMENT_FROM_IMAGE_HPP_

#include <string>
#include <boost/smart_ptr.hpp>
#include <Magick++.h>
#include "sr_movements/partial_movement.hpp"

namespace shadowrobot
{
class MovementFromImage : public PartialMovement
{
public:
  explicit MovementFromImage(std::string image_path);
  virtual ~MovementFromImage();

protected:
  /**
   * Generates a movement from the given png file:
   *  - Reads the image from the top left corner, row by row.
   *  - The first encountered pixel is the target (the scale is
   *    the full range of the joint = the height of the png).
   *
   */
  void generate_movement_();

  /// The image from which the movement is generated
  boost::shared_ptr<Magick::Image> image_;

  /// The number of columns in the image
  ssize_t nb_cols_;
  /// The number of rows in the image
  ssize_t nb_rows_;
};
}  // namespace shadowrobot

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
