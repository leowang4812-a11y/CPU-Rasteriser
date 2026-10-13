#include <iostream>
#include <array>
#include <algorithm>
#include <fstream>
#include <vector>
#include <chrono>
#include <thread>

#include "helpers.hpp"
#include "fenster.h"

using namespace std;

constexpr int SCRW = 200;
constexpr int SCRH = 200;
constexpr float NEARP = 0.1f;
constexpr float FARP  = 100.0f;

using CBuf = array<array<Vec3, SCRW>, SCRH>;
using DBuf = array<array<float, SCRW>, SCRH>;
using Triangle = array<Vec3, 3>;
using Mat3f = Mat3<float>;
using Triangles = vector<Triangle>;

struct Camera {
    Vec3 pos;
    float roll, pitch, yaw;
    float FOV;

    // doesn't include translation
    Mat3f view_mat() const {
        return get_rmatrix(-yaw, -pitch);
    }
    // doesn't include perspective divide
    Mat3f proj_mat() const {
        float aspect = (float)SCRW / SCRH;
        float flen = 1.0f / tan(FOV / 2.0f);    

        return Mat3f(
            flen / aspect, 0,    0,
            0,             flen, 0,
            0,             0,    1
        );
    }
};

struct Object {
    vector<Vec3> vertices;
    vector<int>  indices;
};

int vec_to_colour(Vec3 colour) {
    return ((int)colour.x << 16) | ((int)colour.y << 8) | ((int)colour.z);
}
//NDC from -1 to 1
void draw_triangle(CBuf& cbuffer, DBuf& dbuffer, Triangle tri, const Vec3& colour) {

    for (int i = 0; i < 3; ++i) {
        tri[i].x = tri[i].x * SCRW/2 + SCRW/2;
        tri[i].y = tri[i].y * SCRH/2 + SCRH/2;
    }

    Vec3 normal = Vec3::cross(tri[1] - tri[2], tri[0] - tri[1]);
    float D = normal.x * tri[0].x + normal.y * tri[0].y + normal.z * tri[0].z;

    float max_x = tri[0].x, min_x = tri[0].x, max_y = tri[0].y, min_y = tri[0].y;
    for (int i = 1; i < 3; ++i) {
        if (tri[i].x > max_x) {
            max_x = tri[i].x;
        }
        else if (tri[i].x < min_x) {
            min_x = tri[i].x;
        }
        if (tri[i].y > max_y) {
            max_y = tri[i].y;
        }
        else if (tri[i].y < min_y) {
            min_y = tri[i].y;
        }
    }

    Vec3 edge1 = tri[1] - tri[0];
    Vec3 edge2 = tri[2] - tri[1];
    Vec3 edge3 = tri[0] - tri[2];

    Vec3 P0 = Vec3(max(0.0f, min_x+0.5f), max(0.0f, min_y+0.5f), 0.0f); // z doesn't matter for edge tests

    float c1 = edge1.x * (P0.y - tri[0].y) - edge1.y * (P0.x - tri[0].x);
    float c2 = edge2.x * (P0.y - tri[1].y) - edge2.y * (P0.x - tri[1].x);
    float c3 = edge3.x * (P0.y - tri[2].y) - edge3.y * (P0.x - tri[2].x);

    for (float y = max(0.0f, min_y+0.5f); y < min((float)SCRH, max_y+0.5f); ++y) {
        float row_c1 = c1;
        float row_c2 = c2;
        float row_c3 = c3;
        for (float x = max(0.0f, min_x+0.5f); x < min((float)SCRW, max_x+0.5f); ++x) {
            if (row_c1 > 0 && row_c2 > 0 && row_c3 > 0) {

                float depth = (D - normal.x * x - normal.y * y) / normal.z;
                int scr_y = SCRH - y - 1;
                int scr_x = (int)x;
                if (dbuffer[scr_y][scr_x] > depth) {
                    cbuffer[scr_y][scr_x] = colour * max(0.2f, 1.0f / (1.0f + depth));
                    dbuffer[scr_y][scr_x] = depth;
                }
            }

            row_c1 -= edge1.y;
            row_c2 -= edge2.y;
            row_c3 -= edge3.y;
        }
        c1 += edge1.x;
        c2 += edge2.x;
        c3 += edge3.x;
    }
}

void draw_object(CBuf& cbuffer, DBuf& dbuffer, const Object& obj, const Camera& cam) {
    Mat3f view_proj = cam.proj_mat() * cam.view_mat();
    for (int i = 0; i < obj.indices.size(); i += 3) {
        Triangle tri;

        for (int j = 0; j < 3; ++j) {
            tri[j] = obj.vertices[obj.indices[i+j]];
            tri[j] -= cam.pos;
            tri[j] = view_proj * tri[j];
            
            if (tri[j].z < NEARP || tri[j].z > FARP) {
                return;
            }
        }   
        Triangle tri1{ Vec3{ -0.5, 0.5, 0.5 }, Vec3{ 0.0, 0.0, 0.2 }, Vec3{-0.5, -0.5, 5 } };
        draw_triangle(cbuffer, dbuffer, tri1, Vec3{ 123, 45, 67 });
    }   
}


int main() {
    ofstream fout("img.ppm");

    Triangle tri1{ Vec3{ -0.5, 0.5, 0.5 }, Vec3{ 0.0, 0.0, 0.2 }, Vec3{-0.5, -0.5, 5 } };
    // Triangle tri2{ Vec3{ 0.1, 0.5, 0.1 }, Vec3{ -0.1, -0.4, 8.0 }, Vec3{0.3, -0.4, 0.2 } };

    CBuf cbuffer{};
    DBuf dbuffer{}; for (auto& row : dbuffer) row.fill(1e9);

    Camera camera{ .pos = Vec3{ 0.0f, 0.0f, -2.0f }, .roll = 0.0f, .pitch = 0.0f, .yaw = 0.0f, .FOV = 90.0f };

    Object obj {
        .vertices = {Vec3{1.0f, 1.0f, 1.0f}, Vec3{-1.0f, 1.0f, 1.0f}, Vec3{-1.0f, -1.0f, 1.0f}, Vec3{1.0f, -1.0f, 1.0f}, },
        .indices = { 0, 2, 1, 0, 2, 3 }
    };

    for (auto& v : obj.vertices) {
        v.x *= 0.5f;
        v.y *= 0.5f;
    }




// main renderloop    

    uint32_t* buf = new uint32_t[SCRW * SCRH];
    struct fenster f = { .title = "hello", .width = SCRW, .height = SCRH, .buf = buf};
    fenster_open(&f);

    using namespace std::chrono_literals;
    constexpr auto loop_dur = 200ms;

    bool pressed = false;
    while (fenster_loop(&f) == 0) {
        auto start_t = chrono::steady_clock::now();
        draw_object(cbuffer, dbuffer, obj, camera);

        for (int i = 0; i < SCRH; i++) {
            for (int j = 0; j < SCRW; j++) {
                fenster_pixel(&f, j, i) = vec_to_colour(cbuffer[i][j]);
            }
        }


        if (f.mouse == 1 && !pressed) {
            obj.vertices[0].z -= 0.1f;
            pressed = true;
        }
        if (f.mouse == 0) {
            pressed = false;
        }


        auto next_wake = start_t + loop_dur;
        this_thread::sleep_until(next_wake);
    }

    fenster_close(&f);

    delete[] buf;

    return 0;
}