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

#include "sphere.hpp"

#include "intersection_data.hpp"
#include "../scene/ray.hpp"
#include "lib/common/error_handling.hpp"

#include <cmath>

namespace cpe
{

sphere::sphere(vec3 const& center_param,float radius_param)
    :center_data(center_param),radius_data(radius_param)
{}

vec3 const& sphere::center() const
{
    return center_data;
}
float sphere::radius() const
{
    return radius_data;
}

bool sphere::intersect(ray const& ray_param,intersection_data& intersection) const
{

    vec3 const& u = ray_param.u();

    // definition des coefficient de l'equation du seconde degre
    float a = dot(u,u);
    float b = 2 * dot(u, ray_param.p0()-center_data);
    float c = dot(ray_param.p0()-center_data, ray_param.p0()-center_data) - radius_data * radius_data;

    // discriminant
    float delta = b*b - 4*a*c;

    float t_inter;

    if (delta<0){
        return false;
    }
    else if(delta==0){
        t_inter = - b/(2*a);
        if (t_inter>0){
            vec3 const p_inter = ray_param.p0() + t_inter*u;
            vec3 const n_inter = (p_inter-center_data)/radius_data;
            intersection.set(p_inter,n_inter,t_inter);
            return true;
        } else {
            return false;
        }
    }
    else if (delta>0){

        float x1 = (-b-sqrt(delta))/(2*a);
        float x2 = (-b+sqrt(delta))/(2*a);

        if (x1>=0 && x2>=0){
            float t_inter = std::min(x1, x2);
            if (t_inter>0){
                vec3 const p_inter = ray_param.p0() + t_inter*u;
                vec3 const n_inter = (p_inter-center_data)/radius_data;
                intersection.set(p_inter,n_inter,t_inter);
                return true;
            } else {
                return false;
            }
        }
        else if (x1*x2<0){
            float t_inter = std::max(x1, x2);
            if (t_inter>0){
                vec3 const p_inter = ray_param.p0() + t_inter*u;
                vec3 const n_inter = (p_inter-center_data)/radius_data;
                intersection.set(p_inter,n_inter,t_inter);
                return true;
            } else {
                return false;
            }
        }
        else {
            return false;
        }
    }

    // ********************************************************** //
    // ********************************************************** //
    //  TO DO:
    //    Calcul d'intersection entre un rayon et une plan
    //
    // Variales:
    //  - Position initiale du rayon: ray_param.p0()
    //  - Vecteur directeur unitaire du rayon: u
    //  - Position du centre de la sphere: center_data
    //  - Rayon de la sphere: radius_data
    //
    // Aide de syntaxe:
    //  - Norme au carre d'un vecteur v: float norme=v.norm2();
    //             ou: float norme=v.dot(v);
    //
    // ********************************************************** //
    // ********************************************************** //

}



}
