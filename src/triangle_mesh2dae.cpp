//
// Created by polo on 5/22/22.
//
#include <Eigen/Dense>
#include <pc2mesh/geometry/point_cloud.hpp>
#include <pc2mesh/geometry/triangle_mesh2dae.hpp>
#include <fstream>
#include <string>

namespace pc2mesh::geometry {

    void tri2dae (const PointCloud &pcd, const TriangleMesh &trimesh){

        const auto & point_list = pcd.points;
        const auto & tri_indices_list = trimesh.indices;
        const auto & normal_list = trimesh.triangle_normals;



        std::ofstream outfile;
        outfile.open("test.dae");

        outfile << "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n"
                  "<COLLADA xmlns=\"http://www.collada.org/2005/11/COLLADASchema\" version=\"1.4.1\">\n"
                  "  <asset>\n"
                  "    <contributor>\n"
                  "      <author>Blender User</author>\n"
                  "      <authoring_tool>Blender 2.72.0 commit date:2014-10-21, commit time:11:38, hash:9e963ae</authoring_tool>\n"
                  "    </contributor>\n"
                  "    <created>2015-09-12T16:43:42</created>\n"
                  "    <modified>2015-09-12T16:43:42</modified>\n"
                  "    <unit name=\"meter\" meter=\"1\"/>\n"
                  "    <up_axis>Z_UP</up_axis>\n"
                  "  </asset>\n"
                  "  <library_cameras>\n"
                  "    <camera id=\"Camera-camera\" name=\"Camera\">\n"
                  "      <optics>\n"
                  "        <technique_common>\n"
                  "          <perspective>\n"
                  "            <xfov sid=\"xfov\">49.13434</xfov>\n"
                  "            <aspect_ratio>1.777778</aspect_ratio>\n"
                  "            <znear sid=\"znear\">0.1</znear>\n"
                  "            <zfar sid=\"zfar\">100</zfar>\n"
                  "          </perspective>\n"
                  "        </technique_common>\n"
                  "      </optics>\n"
                  "      <extra>\n"
                  "        <technique profile=\"blender\">\n"
                  "          <YF_dofdist>0</YF_dofdist>\n"
                  "          <shiftx>0</shiftx>\n"
                  "          <shifty>0</shifty>\n"
                  "        </technique>\n"
                  "      </extra>\n"
                  "    </camera>\n"
                  "  </library_cameras>\n"
                  "  <library_lights>\n"
                  "    <light id=\"Lamp-light\" name=\"Lamp\">\n"
                  "      <technique_common>\n"
                  "        <point>\n"
                  "          <color sid=\"color\">1 1 1</color>\n"
                  "          <constant_attenuation>1</constant_attenuation>\n"
                  "          <linear_attenuation>0</linear_attenuation>\n"
                  "          <quadratic_attenuation>0.00111109</quadratic_attenuation>\n"
                  "        </point>\n"
                  "      </technique_common>\n"
                  "      <extra>\n"
                  "        <technique profile=\"blender\">\n"
                  "          <adapt_thresh>0.000999987</adapt_thresh>\n"
                  "          <area_shape>1</area_shape>\n"
                  "          <area_size>0.1</area_size>\n"
                  "          <area_sizey>0.1</area_sizey>\n"
                  "          <area_sizez>1</area_sizez>\n"
                  "          <atm_distance_factor>1</atm_distance_factor>\n"
                  "          <atm_extinction_factor>1</atm_extinction_factor>\n"
                  "          <atm_turbidity>2</atm_turbidity>\n"
                  "          <att1>0</att1>\n"
                  "          <att2>1</att2>\n"
                  "          <backscattered_light>1</backscattered_light>\n"
                  "          <bias>1</bias>\n"
                  "          <blue>1</blue>\n"
                  "          <buffers>1</buffers>\n"
                  "          <bufflag>0</bufflag>\n"
                  "          <bufsize>2880</bufsize>\n"
                  "          <buftype>2</buftype>\n"
                  "          <clipend>30.002</clipend>\n"
                  "          <clipsta>1.000799</clipsta>\n"
                  "          <compressthresh>0.04999995</compressthresh>\n"
                  "          <dist sid=\"blender_dist\">29.99998</dist>\n"
                  "          <energy sid=\"blender_energy\">1</energy>\n"
                  "          <falloff_type>2</falloff_type>\n"
                  "          <filtertype>0</filtertype>\n"
                  "          <flag>0</flag>\n"
                  "          <gamma sid=\"blender_gamma\">1</gamma>\n"
                  "          <green>1</green>\n"
                  "          <halo_intensity sid=\"blnder_halo_intensity\">1</halo_intensity>\n"
                  "          <horizon_brightness>1</horizon_brightness>\n"
                  "          <mode>8192</mode>\n"
                  "          <ray_samp>1</ray_samp>\n"
                  "          <ray_samp_method>1</ray_samp_method>\n"
                  "          <ray_samp_type>0</ray_samp_type>\n"
                  "          <ray_sampy>1</ray_sampy>\n"
                  "          <ray_sampz>1</ray_sampz>\n"
                  "          <red>1</red>\n"
                  "          <samp>3</samp>\n"
                  "          <shadhalostep>0</shadhalostep>\n"
                  "          <shadow_b sid=\"blender_shadow_b\">0</shadow_b>\n"
                  "          <shadow_g sid=\"blender_shadow_g\">0</shadow_g>\n"
                  "          <shadow_r sid=\"blender_shadow_r\">0</shadow_r>\n"
                  "          <sky_colorspace>0</sky_colorspace>\n"
                  "          <sky_exposure>1</sky_exposure>\n"
                  "          <skyblendfac>1</skyblendfac>\n"
                  "          <skyblendtype>1</skyblendtype>\n"
                  "          <soft>3</soft>\n"
                  "          <spotblend>0.15</spotblend>\n"
                  "          <spotsize>75</spotsize>\n"
                  "          <spread>1</spread>\n"
                  "          <sun_brightness>1</sun_brightness>\n"
                  "          <sun_effect_type>0</sun_effect_type>\n"
                  "          <sun_intensity>1</sun_intensity>\n"
                  "          <sun_size>1</sun_size>\n"
                  "          <type>0</type>\n"
                  "        </technique>\n"
                  "      </extra>\n"
                  "    </light>\n"
                  "  </library_lights>\n"
                  "  <library_images/>\n"
                  "  <library_effects>\n"
                  "    <effect id=\"Material-effect\">\n"
                  "      <profile_COMMON>\n"
                  "        <technique sid=\"common\">\n"
                  "          <phong>\n"
                  "            <emission>\n"
                  "              <color sid=\"emission\">0 0 0 1</color>\n"
                  "            </emission>\n"
                  "            <ambient>\n"
                  "              <color sid=\"ambient\">0 0 0 1</color>\n"
                  "            </ambient>\n"
                  "            <diffuse>\n"
                  "              <color sid=\"diffuse\">0.64 0.64 0.64 1</color>\n"
                  "            </diffuse>\n"
                  "            <specular>\n"
                  "              <color sid=\"specular\">0.5 0.5 0.5 1</color>\n"
                  "            </specular>\n"
                  "            <shininess>\n"
                  "              <float sid=\"shininess\">50</float>\n"
                  "            </shininess>\n"
                  "            <index_of_refraction>\n"
                  "              <float sid=\"index_of_refraction\">1</float>\n"
                  "            </index_of_refraction>\n"
                  "          </phong>\n"
                  "        </technique>\n"
                  "      </profile_COMMON>\n"
                  "    </effect>\n"
                  "  </library_effects>\n"
                  "  <library_materials>\n"
                  "    <material id=\"Material-material\" name=\"Material\">\n"
                  "      <instance_effect url=\"#Material-effect\"/>\n"
                  "    </material>\n"
                  "  </library_materials>\n"
                  "  <library_geometries>\n"
                  "    <geometry id=\"Cube-mesh\" name=\"Cube\">\n"
                  "      <mesh>\n"
                  "        <source id=\"Cube-mesh-positions\">";


        outfile << "<float_array id=\"Cube-mesh-positions-array\" count=\""
                << point_list.size()*3
                << "\">"
                << std::endl;

        for (const auto & point: point_list){
                outfile << "\t" << point[0] << " " <<  std::to_string(point[1]) << " " << std::to_string(point[2]) << std::endl;
        }



        outfile << "</float_array>" << std::endl;


        outfile << "<technique_common>\n"
                  "            <accessor source=\"#Cube-mesh-positions-array\" count=\"8\" stride=\"3\">\n"
                  "              <param name=\"X\" type=\"float\"/>\n"
                  "              <param name=\"Y\" type=\"float\"/>\n"
                  "              <param name=\"Z\" type=\"float\"/>\n"
                  "            </accessor>\n"
                  "          </technique_common>\n"
                  "        </source>\n"
                  "        <source id=\"Cube-mesh-normals\">\n"
                  "          <float_array id=\"Cube-mesh-normals-array\" count=\"" << normal_list.size() <<"\">";
        for (const auto & norm: normal_list){
            outfile << "\t" << norm[0] << " " <<  norm[1] << " " << norm[2] << std::endl;
        }

        outfile << "</float_array>\n"
                  "          <technique_common>\n"
                  "            <accessor source=\"#Cube-mesh-normals-array\" count=\"12\" stride=\"3\">\n"
                  "              <param name=\"X\" type=\"float\"/>\n"
                  "              <param name=\"Y\" type=\"float\"/>\n"
                  "              <param name=\"Z\" type=\"float\"/>\n"
                  "            </accessor>\n"
                  "          </technique_common>\n"
                  "        </source>\n"
                  "        <vertices id=\"Cube-mesh-vertices\">\n"
                  "          <input semantic=\"POSITION\" source=\"#Cube-mesh-positions\"/>\n"
                  "        </vertices>\n"
                  "        <polylist material=\"Material-material\" count=\""+std::to_string(tri_indices_list.size())+"\">\n"
                  "          <input semantic=\"VERTEX\" source=\"#Cube-mesh-vertices\" offset=\"0\"/>\n"
                  "          <input semantic=\"NORMAL\" source=\"#Cube-mesh-normals\" offset=\"1\"/>\n"
                  "          <vcount>";
        auto buffer = std::string(tri_indices_list.size() * 2, '3');
        for (size_t i = 1; i < buffer.size(); i += 2) {
            buffer[i] = ' ';
        }
        outfile << '\t' << buffer << std::endl;

        outfile << "</vcount>\n";
        outfile << "          <p>";

        int counter = 0;
        for (const auto & index: tri_indices_list){
            outfile << index[0]<<" "<< counter << " "<< index[1] << " " <<counter  <<" " << index[2] << " "     <<counter << std::endl;
            counter ++;
        }

        outfile << "</p>\n"
                  "        </polylist>\n"
                  "      </mesh>\n"
                  "    </geometry>\n"
                  "  </library_geometries>\n"
                  "  <library_controllers/>\n"
                  "  <library_visual_scenes>\n"
                  "    <visual_scene id=\"Scene\" name=\"Scene\">\n"
                  "      <node id=\"Camera\" name=\"Camera\" type=\"NODE\">\n"
                  "        <matrix sid=\"transform\">0.6858805 -0.3173701 0.6548619 7.481132 0.7276338 0.3124686 -0.6106656 -6.50764 -0.01081678 0.8953432 0.4452454 5.343665 0 0 0 1</matrix>\n"
                  "        <instance_camera url=\"#Camera-camera\"/>\n"
                  "      </node>\n"
                  "      <node id=\"Lamp\" name=\"Lamp\" type=\"NODE\">\n"
                  "        <matrix sid=\"transform\">-0.2908646 -0.7711008 0.5663932 4.076245 0.9551712 -0.1998834 0.2183912 1.005454 -0.05518906 0.6045247 0.7946723 5.903862 0 0 0 1</matrix>\n"
                  "        <instance_light url=\"#Lamp-light\"/>\n"
                  "      </node>\n"
                  "      <node id=\"Cube\" name=\"Cube\" type=\"NODE\">\n"
                  "        <matrix sid=\"transform\">1.999999 0 0 0 0 2 0 0 0 0 2 0 0 0 0 1</matrix>\n"
                  "        <instance_geometry url=\"#Cube-mesh\">\n"
                  "          <bind_material>\n"
                  "            <technique_common>\n"
                  "              <instance_material symbol=\"Material-material\" target=\"#Material-material\"/>\n"
                  "            </technique_common>\n"
                  "          </bind_material>\n"
                  "        </instance_geometry>\n"
                  "      </node>\n"
                  "    </visual_scene>\n"
                  "  </library_visual_scenes>\n"
                  "  <scene>\n"
                  "    <instance_visual_scene url=\"#Scene\"/>\n"
                  "  </scene>\n"
                  "</COLLADA>";



        outfile.flush();

    }


}