#include <iostream>
#include <fstream>
#include <tgmath.h>
#include <pcg32.h>
#include "imedit/image.h"
#include "imedit/im_util.h"
#include "imedit/procedural.h"
#include "imedit/filter.h"
#include "imedit/im_color_maps.h"
#include <string>
#include <filesystem>

void init_base_scene(imedit::Image &image)
{
    for (int i = 0; i < 17; ++i)
    {
        image(0, i) = imedit::Pixel(1.0);
        if (i != 12)
            image(8, i) = imedit::Pixel(1.0);
        image(16, i) = imedit::Pixel(1.0);
        if (i != 2 && i != 6 && i != 10 && i != 14)
        {
            image(4, i) = imedit::Pixel(1.0);
            image(12, i) = imedit::Pixel(1.0);
        }
    }

    for (int j = 0; j < 17; ++j)
    {
        image(j, 0) = imedit::Pixel(1.0);
        image(j, 16) = imedit::Pixel(1.0);
        if (j < 4 || j > 12)
        {
            image(j, 4) = imedit::Pixel(1.0);
            image(j, 8) = imedit::Pixel(1.0);
            image(j, 12) = imedit::Pixel(1.0);
        }
    }
}

imedit::Image detect_corner(imedit::Image &scene)
{
    // imedit::Image image = scene;

    // imedit::Image xgrad = image;
    // imedit::Image ygrad = image;

    // imedit::xgrad3x1(scene, xgrad);
    // imedit::ygrad1x3(scene, ygrad);

    // imedit::Image sum = xgrad + ygrad;

    // imedit::negpos_from_gray(xgrad);
    // imedit::negpos_from_gray(ygrad);
    // imedit::negpos_from_gray(sum);

    // xgrad.write("xgrad.png");
    // ygrad.write("ygrad.png");
    // sum.write("grad.png");

    // TODO: make smarter with the Harris function

    // return image;

    // implementing naively dumb method here
    imedit::Image image = scene;
    for (int i = 0; i < image.height(); ++i)
    {
        std::cout << i << std::endl;
        for (int j = 0; j < image.width(); ++j)
        {
            image(j, i) = imedit::Pixel(0.0);

            imedit::Pixel center = scene.filter_index(j, i);
            if (center.r > 0.0)
            {
                std::cout << "PRE" << std::endl;
                imedit::Pixel one = scene.filter_index(j + 1, i);
                imedit::Pixel two = scene.filter_index(j - 1, i);
                imedit::Pixel three = scene.filter_index(j, i + 1);
                imedit::Pixel four = scene.filter_index(j, i - 1);
                std::cout << "POST" << std::endl;

                int num_empty_x = 0;
                int num_empty_y = 0;

                if (one.r == 0.0)
                    num_empty_x++;
                if (two.r == 0.0)
                    num_empty_x++;
                if (three.r == 0.0)
                    num_empty_y++;
                if (four.r == 0.0)
                    num_empty_y++;

                if (num_empty_x > 0 && num_empty_y > 0)
                    image(j, i) = imedit::Pixel(1.0, 0.0, 0.0);
            }
        }
    }

    return image;
}

int dist_test(int pj, int mj, int pi, int mi)
{
    if (pj == 0.0)
        pj = 1000;
    if (mj == 0.0)
        mj = 1000;
    if (pi == 0.0)
        pi = 1000;
    if (mi == 0.0)
        mi = 1000;

    if (pj <= mj && pj <= pi && pj <= mi)
        return 0;
    if (mj <= pj && mj <= pi && mj <= mi)
        return 1;
    if (pi <= pj && pi <= mj && pi <= mi)
        return 2;
    if (mi <= pi && mi <= mj && mi <= pj)
        return 3;
    return -1;
}

imedit::Image create_important_points(imedit::Image &scene, imedit::Image &corners)
{
    imedit::Image image = scene;

    for (int i = 0; i < image.height(); ++i)
    {
        for (int j = 0; j < image.width(); ++j)
        {
            image(j, i) = imedit::Pixel(0.0);
        }
    }

    for (int i = 0; i < corners.height(); ++i)
    // for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < corners.width(); ++j)
        {
            if (corners(j, i).r > 0.0)
            {
                // find right collision
                int rj = j + 1;
                while (scene(rj, i).r == 0.0)
                    rj++;
                // find left collision
                int lj = j - 1;
                while (scene(lj, i).r == 0.0)
                    lj--;
                // find up collision
                int ri = i + 1;
                while (scene(j, ri).r == 0.0)
                    ri++;
                // std::cout << "pre ri: " << ri << std::endl;
                // find down collision
                int li = i - 1;
                while (scene(j, li).r == 0.0)
                    li--;

                // std::cout << "collision rj: " << rj << std::endl;
                // std::cout << "collision lj: " << lj << std::endl;
                // std::cout << "collision ri: " << ri << std::endl;
                // std::cout << "collision li: " << li << std::endl;

                int pj_dist = std::abs(rj - j) - 1;
                int mj_dist = std::abs(lj - j) - 1;
                int pi_dist = std::abs(ri - i) - 1;
                int mi_dist = std::abs(li - i) - 1;

                // std::cout << "collision rj_dist: " << pj_dist << std::endl;
                // std::cout << "collision lj_dist: " << mj_dist << std::endl;
                // std::cout << "collision ri_dist: " << pi_dist << std::endl;
                // std::cout << "collision li_dist: " << mi_dist << std::endl;

                // std::cout << "ACK" << std::endl;
                int res = dist_test(pj_dist, mj_dist, pi_dist, mi_dist);

                if (res == 0)
                {
                    // std::cout << "RJ: " << rj << " j: " << j << std::endl;
                    image((rj + j) / 2, i) = imedit::Pixel(0.0, 0.0, 1.0);
                }
                if (res == 1)
                {
                    // std::cout << "LJ: " << lj << std::endl;
                    image((lj + j) / 2, i) = imedit::Pixel(0.0, 0.0, 1.0);
                }
                if (res == 2)
                {
                    // std::cout << "ri: " << ri << std::endl;
                    image(j, (ri + i) / 2) = imedit::Pixel(0.0, 0.0, 1.0);
                }
                if (res == 3)
                {
                    // std::cout << "li: " << li << std::endl;
                    image(j, (li + i) / 2) = imedit::Pixel(0.0, 0.0, 1.0);
                }
            }
        }
    }

    return image;
}

imedit::Image create_nodes(imedit::Image &scene, imedit::Image &imports)
{
    imedit::Image image = scene;

    for (int i = 0; i < image.height(); ++i)
    {
        for (int j = 0; j < image.width(); ++j)
        {
            image(j, i) = imedit::Pixel(0.0);
        }
    }

    for (int i = 0; i < imports.height(); ++i)
    {
        for (int j = 0; j < imports.width(); ++j)
        {
            if (imports(j, i).b > 0.0)
            {
                // find right collision
                int rj = j + 1;
                while (scene(rj, i).r == 0.0)
                    rj++;
                // find left collision
                int lj = j - 1;
                while (scene(lj, i).r == 0.0)
                    lj--;
                // find up collision
                int ri = i + 1;
                while (scene(j, ri).r == 0.0)
                    ri++;
                // std::cout << "pre ri: " << ri << std::endl;
                // find down collision
                int li = i - 1;
                while (scene(j, li).r == 0.0)
                    li--;

                // std::cout << "collision rj: " << rj << std::endl;
                // std::cout << "collision lj: " << lj << std::endl;
                // std::cout << "collision ri: " << ri << std::endl;
                // std::cout << "collision li: " << li << std::endl;

                int pj_dist = std::abs(rj - j) - 1;
                int mj_dist = std::abs(lj - j) - 1;
                int pi_dist = std::abs(ri - i) - 1;
                int mi_dist = std::abs(li - i) - 1;

                // std::cout << "collision rj_dist: " << pj_dist << std::endl;
                // std::cout << "collision lj_dist: " << mj_dist << std::endl;
                // std::cout << "collision ri_dist: " << pi_dist << std::endl;
                // std::cout << "collision li_dist: " << mi_dist << std::endl;

                // std::cout << "ACK" << std::endl;
                // int res = dist_test(pj_dist, mj_dist, pi_dist, mi_dist);

                if (pj_dist > 0)
                {
                    image((rj + j) / 2, i) = imedit::Pixel(0.0, 1.0, 0.0);
                }
                if (mj_dist > 0)
                {
                    image((lj + j) / 2, i) = imedit::Pixel(0.0, 1.0, 0.0);
                }
                if (pi_dist > 0)
                {
                    // std::cout << "ri: " << ri << std::endl;
                    image(j, (ri + i) / 2) = imedit::Pixel(0.0, 1.0, 0.0);
                }
                if (mi_dist > 0)
                {
                    // std::cout << "li: " << li << std::endl;
                    image(j, (li + i) / 2) = imedit::Pixel(0.0, 1.0, 0.0);
                }
            }
        }
    }

    return image;
}

imedit::Image connect_nodes(imedit::Image &scene, imedit::Image &nodes)
{
    imedit::Image image = scene;

    for (int i = 0; i < image.height(); ++i)
    {
        for (int j = 0; j < image.width(); ++j)
        {
            image(j, i) = imedit::Pixel(0.0);
        }
    }

    for (int i = 0; i < nodes.height(); ++i)
    {
        for (int j = 0; j < nodes.width(); ++j)
        {
            if (nodes(j, i).g > 0.0)
            {
                // try right connection
                int rj = j + 1;
                while (scene(rj, i).r == 0.0)
                {
                    if (nodes(rj, i).g > 0.0)
                    {
                        for (int k = j; k <= rj; k++)
                        {
                            image(k, i) = imedit::Pixel(0.0, 1.0, 1.0);
                        }
                    }
                    rj++;
                }
                // find left collision
                int lj = j - 1;
                while (scene(lj, i).r == 0.0)
                {
                    if (nodes(lj, i).g > 0.0)
                    {
                        for (int k = lj; k <= j; k++)
                        {
                            image(k, i) = imedit::Pixel(0.0, 1.0, 1.0);
                        }
                    }
                    lj--;
                }
                // find up collision
                int ri = i + 1;
                while (scene(j, ri).r == 0.0)
                {
                    if (nodes(j, ri).g > 0.0)
                    {
                        for (int k = i; k <= ri; k++)
                        {
                            image(j, k) = imedit::Pixel(0.0, 1.0, 1.0);
                        }
                    }
                    ri++;
                }
                // std::cout << "pre ri: " << ri << std::endl;
                // find down collision
                int li = i - 1;
                while (scene(j, li).r == 0.0)
                {
                    if (nodes(j, ri).g > 0.0)
                    {
                        for (int k = li; k <= i; k++)
                        {
                            image(j, k) = imedit::Pixel(0.0, 1.0, 1.0);
                        }
                    }
                    li--;
                }

                // // std::cout << "collision rj: " << rj << std::endl;
                // // std::cout << "collision lj: " << lj << std::endl;
                // // std::cout << "collision ri: " << ri << std::endl;
                // // std::cout << "collision li: " << li << std::endl;

                // int pj_dist = std::abs(rj - j) - 1;
                // int mj_dist = std::abs(lj - j) - 1;
                // int pi_dist = std::abs(ri - i) - 1;
                // int mi_dist = std::abs(li - i) - 1;

                // // std::cout << "collision rj_dist: " << pj_dist << std::endl;
                // // std::cout << "collision lj_dist: " << mj_dist << std::endl;
                // // std::cout << "collision ri_dist: " << pi_dist << std::endl;
                // // std::cout << "collision li_dist: " << mi_dist << std::endl;

                // // std::cout << "ACK" << std::endl;
                // // int res = dist_test(pj_dist, mj_dist, pi_dist, mi_dist);

                // if (pj_dist > 0)
                // {
                //     image((rj + j) / 2, i) = imedit::Pixel(0.0, 1.0, 0.0);
                // }
                // if (mj_dist > 0)
                // {
                //     image((lj + j) / 2, i) = imedit::Pixel(0.0, 1.0, 0.0);
                // }
                // if (pi_dist > 0)
                // {
                //     // std::cout << "ri: " << ri << std::endl;
                //     image(j, (ri + i) / 2) = imedit::Pixel(0.0, 1.0, 0.0);
                // }
                // if (mi_dist > 0)
                // {
                //     // std::cout << "li: " << li << std::endl;
                //     image(j, (li + i) / 2) = imedit::Pixel(0.0, 1.0, 0.0);
                // }
            }
        }
    }

    return image;
}

imedit::Image invert_image(imedit::Image &scene)
{
    imedit::Image image = scene;

    for (int i = 0; i < scene.height(); ++i)
    {
        for (int j = 0; j < scene.width(); ++j)
        {
            if (scene(j, i).r > 0.5)
            {
                image(j, i) = imedit::Pixel(0.0);
            }
            else
            {
                image(j, i) = imedit::Pixel(1.0);
            }
        }
    }

    return image;
}

int main(int argc, char *argv[])
{
    // std::filesystem::path cwd = std::filesystem::current_path();
    // printf("cwd: %s\n", cwd.string().c_str());
    // std::string path = "/home/wojciechs-lab/robotics_final/src/cs69-final-project/completed_maps/map.png";
    std::string path = "base_map_clean.png";
    printf("loading image %s\n", path.c_str());
    imedit::Image base_scene = imedit::Image(path);
    printf("loaded image\n");
    imedit::Image inverted = invert_image(base_scene);
    // init_base_scene(base_scene);
    base_scene.write("nodegen_base.png");
    inverted.write("inverted_base.png");
    // printf("init\n");
    imedit::Image corner_image = detect_corner(inverted);
    printf("corner\n");
    corner_image.write("nodegen_corners.png");

    imedit::Image imp_pts = create_important_points(inverted, corner_image);
    printf("imp\n");
    imp_pts.write("important_pts.png");

    imedit::Image nodes = create_nodes(inverted, imp_pts);
    printf("node\n");
    // nodes.write("nodes.png");

    imedit::Image node_graph = connect_nodes(inverted, nodes);
    printf("graph\n");
    // base_scene = imedit::mult_size(base_scene, 64);
    // corner_image = imedit::mult_size(corner_image, 64);
    // imp_pts = imedit::mult_size(imp_pts, 64);
    // nodes = imedit::mult_size(nodes, 64);
    // node_graph = imedit::mult_size(node_graph, 64);

    // base_scene.write("nodegen_base.png");
    corner_image.write("nodegen_corners.png");
    imp_pts.write("important_pts.png");
    nodes.write("nodes.png");
    node_graph.write("node_graph.png");

    imedit::Image scene_corners = (base_scene + corner_image * 3.0) / 4.0;
    scene_corners.write("scene_p_corners.png");

    imedit::Image corners_imp_pts = (scene_corners + imp_pts * 3.0) / 4.0;
    corners_imp_pts.write("corners_imp_pts.png");

    imedit::Image imp_pts_nodes = (corners_imp_pts + nodes * 3.0) / 4.0;
    imp_pts_nodes.write("imp_pts_nodes.png");

    imedit::Image nodes_node_graph = (imp_pts_nodes * 4.0 + node_graph * 1.0) / 5.0;
    nodes_node_graph.write("nodes_node_graph.png");

    imedit::Image final_graph = base_scene + node_graph;
    final_graph.write("final_graph.png");

    return 0;

    // std::ofstream image_code;
    // image_code.open("image_code.txt");

    // for (int k = 0; k < 8; ++k)
    // {
    //     std::string image_path = "/Users/corneria/Documents/Art/sonic_run/run_" + std::to_string(k) + ".png";

    //     imedit::Image image = imedit::Image(image_path);

    //     image_code << "vec3 image_" << k << "(vec2 uv, float time)" << std::endl;
    //     image_code << "{" << std::endl;
    //     image_code << "    uv.x -= 0.25;" << std::endl;
    //     image_code << "    uv.x *= aspect;" << std::endl;
    //     image_code << "    ivec2 pixel_index = ivec2(int(floor(uv.x * 50.f * 1.f)), int(floor(50.f - uv.y * 50.f)));" << std::endl;
    //     image_code << "    vec3 color = vec3(0.0);" << std::endl;

    //     for (int i = 0; i < image.height(); i++)
    //     {
    //         for (int j = 0; j < image.height(); j++)
    //         {
    //             imedit::Pixel pix = image(j, i);

    //             if (pix != imedit::Pixel(1.f))
    //             {
    //                 image_code << "    color += chk(pixel_index, ivec2(" << j << ", " << i << ")) * (vec3(" << pix.r << ", " << pix.g << ", " << pix.b << "));" << std::endl;
    //             }
    //         }
    //     }

    //     image_code << "    return color;" << std::endl;
    //     image_code << "}" << std::endl;
    // }

    // return 0;
}
