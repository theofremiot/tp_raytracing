/*
**    TP CPE Lyon
**    Copyright (C) 2015 Damien Rohmer
**
**    This program is free software: you can redistribute it and/or modify
**    it under the terms of the GNU General Public License as published by
**    the Free Software Foundation, either version 3 of the License, or
**    (at your option) any later version.
**
**   This program is distributed in the hope that it will be useful,
**    but WITHOUT ANY WARRANTY; without even the implied warranty of
**    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
**    GNU General Public License for more details.
**
**    You should have received a copy of the GNU General Public License
**    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "plane.hpp"
#include "intersection_data.hpp"
#include "../scene/ray.hpp"

#include <cmath>

namespace cpe
{

plane::plane(vec3 const& position_param,vec3 const& normal_param)
    :position_data(position_param),normal_data(normal_param)
{

}

vec3 const& plane::position() const
{
    return position_data;
}
vec3 const& plane::normal() const
{
    return normal_data;
}

bool plane::intersect(ray const& ray_param,intersection_data& intersection) const
{

    vec3 const& u = ray_param.u();
    vec3 const& n = normal_data;

    if (dot(u,n)==0.0f){
        return false;
    }
    else{

        float const t_inter = (dot(position_data-ray_param.p0(), n))/(dot(u,n));
        if (t_inter>0){
            vec3 const p_inter = ray_param.p0() + t_inter*u;
            vec3 const n_inter = n;

            intersection.set(p_inter,n_inter,t_inter);
            return true;
        } else {
            return false;
        }
    }

}

}
