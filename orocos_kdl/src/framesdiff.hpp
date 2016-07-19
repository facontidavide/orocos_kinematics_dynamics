/***************************************************************************
                        framesdiff.hpp -  description
                       -------------------------
    begin                : July 2016
    author               : Davide Faconti
    email                : davide.faconti@gmail.com

 ***************************************************************************
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Lesser General Public            *
 *   License as published by the Free Software Foundation; either          *
 *   version 2.1 of the License, or (at your option) any later version.    *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 *                                                                         *
 ***************************************************************************/

#ifndef KDL_FRAMESDIFF_H
#define KDL_FRAMESDIFF_H

#include <frames.hpp>
#include <Eigen/Core>

namespace KDL{

/**
 * This funtion is meant rotation difference between two frames,
 * but ignoring the component parallel to a specified axis.
 *
 * @param Ra  first rotation
 * @param Rb  second rotation
 * @param neutral_rot_axis  this is the vector used to "neutralize" the errors parallel to the axis.
 * @return difference vector.
 */

IMETHOD Vector diff_2DoF(const Rotation& Ra,const Rotation& Rb, const Vector& neutral_rot_axis )
{
    // rotation from the frame A to frame B
    Rotation R_a_b    = Ra.Inverse()*Rb;

    // extract the rotation axis as it is seen from the point of view of Ra
    Vector rot_axis;
    double angle = R_a_b.GetRotAngle(rot_axis);

    // project rot_axis onto neutral_rot_axis and subtract the result from rot_axis itself
    // in this way the perpendicular components will be maintained.
    Vector proj_axis = neutral_rot_axis * dot( neutral_rot_axis, rot_axis);

    return Ra * ( (rot_axis - proj_axis) * angle );
}


/**
 * This funtion is meant rotation and position difference between two frames,
 * but ignoring the rotation part that is parallel to a specified axis.
 *
 * @param Fa  first frame
 * @param Fb  second frame
 * @param neutral_rot_axis  this is the vector used to "neutralize" the errors parallel to the axis.
 * @return difference represented as a 6 dimensional vector. We use a Twist, even if technically it is not a twist.
 */

IMETHOD Twist diff_5DoF(const Frame& Fa,const Frame& Fb,
                        const Vector& neutral_rot_axis)
{
    return Twist ( (Fa.p - Fb.p),
                  diff_2DoF( Fa.M, Fb.M, neutral_rot_axis) );
}



}

#endif
