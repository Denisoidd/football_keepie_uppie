
#include "00_introduction.hpp"

#include <random>

#ifdef INF443_INTRODUCTION
// Add vcl namespace within the current one - Allows to use function from vcl library without explicitely preceeding their name with vcl::
using namespace vcl;

// Generator for uniform random number
std::default_random_engine generator;
std::uniform_real_distribution<float> distrib(0.0,1.0);


float evaluate_terrain_z(float u, float v);
vec3 evaluate_terrain(float u, float v);
mesh create_terrain();
mesh create_tree_foliage(float radius, float height, float z_offset);

mesh create_corners_stade(float scale, float smallSide, float bigSide, float x_centre, float y_centre);
mesh create_supporteurs(float scale, float smallSide, float bigSide, float x_centre, float y_centre);
mesh create_cone(float radius, float height, float z_offset);

vcl::mesh create_skybox();

static size_t index_at_value(float t, const std::vector<float>& vt)
{
    const size_t N = vt.size();
    assert(vt.size()>=2);
    assert(t>=vt[0]);
    assert(t<vt[N-1]);

    size_t k=0;
    while( vt[k+1]<t )
        ++k;
    return k;
}
static vec3 linear_interpolation(float t, float t1, float t2, const vec3& p1, const vec3& p2)
{
    const float alpha = (t-t1)/(t2-t1);
    const vec3 p = (1-alpha)*p1 + alpha*p2;

    return p;
}
void scene_exercise::setup_data(std::map<std::string,GLuint>& , scene_structure& scene, gui_structure& )
{
//-------------------------------------------------------------CONSTANSTS FOR BODY CONSTRUCTION-------------------------------------------------------------------------------//
    const float r_body = 0.15f*0.7f;
    const float r_cylinder = 0.075f*0.7f;
    const float l_anche = 0.08f*0.7f;
    const float r_pied = 0.080f*0.7f;
    const float l_pied = 0.3f*0.7f;
    const float l_arm = 0.45f*0.7f;
    const float r_sph = 0.08f*0.7f;
    const float r_balon = 1.5f*r_sph*0.7f;
    const float r_head = 1.8f*r_sph;
    const float l_body = 0.7f*0.7f;
    const float l_epo = -l_arm*0.65f*0.7f;
    const float r_hand = r_cylinder*0.75f*0.7f;
    const float delta = l_body*1.5f+0.02f;

//-------------------------------------------------------------CONSTANTS FOR THE MOVEMENT OF THE BALL------------------------------------------------------------------------//
    const float N_parties = 4;
    const float h_begin = 3*r_pied;
    const float w_begin = -l_pied*1.5f;
    const float l_begin = -2.5f*l_anche;
    const float l_end = 2.5f*l_anche;
    const float step = (l_end-l_begin)/(N_parties-1);
    const float coef_h1 = 4.5f;
    const float coef_h2 = 6.0f;
    keyframe_position = {{l_begin,w_begin,h_begin}, {l_begin + step*1.0f,w_begin,h_begin*coef_h1}, {l_begin + step*1.0f*2,w_begin,h_begin*coef_h2}, {l_begin + step*1.0f*3,w_begin,h_begin*coef_h1}, {l_end,w_begin, h_begin}, {l_begin + step*1.0f*3,w_begin,h_begin*coef_h1}, {l_begin + step*1.0f*2,w_begin,h_begin*coef_h2}, {l_begin + step*1.0f,w_begin,h_begin*coef_h1}, {l_begin,w_begin,h_begin}, {l_begin + step*1.0f,w_begin,h_begin*coef_h1},{l_begin + step*1.0f*2,w_begin,h_begin*coef_h2}, {l_begin + step*1.0f*3,w_begin,h_begin*coef_h1}, {l_end,w_begin, h_begin}, {l_begin + step*1.0f*3,w_begin,h_begin*coef_h1}, {l_begin + step*1.0f*2,w_begin,h_begin*coef_h2}, {l_begin + step*1.0f,w_begin,h_begin*coef_h1}, {l_begin,w_begin,h_begin}, {l_begin + step*1.0f,w_begin,h_begin*coef_h1}, {l_begin + step*1.0f*2,w_begin,h_begin*coef_h2}, {l_begin + step*1.0f*3,w_begin,h_begin*coef_h1}, {l_end,w_begin, h_begin}, {l_begin + step*1.0f*3,w_begin,h_begin*coef_h1}, {l_begin + step*1.0f*2,w_begin,h_begin*coef_h2}, {l_begin + step*1.0f,w_begin,h_begin*coef_h1}, {l_begin,w_begin,h_begin}, {l_begin + step*1.0f,w_begin,h_begin*coef_h1},{l_begin + step*1.0f*2,w_begin,h_begin*coef_h2}, {l_begin + step*1.0f*3,w_begin,h_begin*coef_h1}, {l_end,w_begin, h_begin}, {l_begin + step*1.0f*3,w_begin,h_begin*coef_h1}, {l_begin + step*1.0f*2,w_begin,h_begin*coef_h2}, {l_begin + step*1.0f,w_begin,h_begin*coef_h1}, {l_begin,w_begin,h_begin}, {l_begin + step*1.0f,w_begin,h_begin*coef_h1}};
    keyframe_time = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17};
    for (int i=0; i < keyframe_time.size(); i++){
       keyframe_time[i] = keyframe_time[i]/5.0f;
    }
     // Set timer bounds
     //  To ease spline interpolation of a closed curve time \in [t_1,t_{N-2}]
     timer.t_min = keyframe_time[1];
     timer.t_max = keyframe_time[keyframe_time.size()-2];
     timer.t = timer.t_min;
     // Prepare the visual elements
     surface0 = mesh_primitive_sphere();
     surface0.uniform_parameter.color   = {0.2f,0.2f,0.2f};
     surface0.uniform_parameter.scaling = 0.08f;

     sphere0 = mesh_primitive_sphere();
     sphere0.uniform_parameter.color = {1,1,1};
     sphere0.uniform_parameter.scaling = 0.05f;

     segment_drawer.init();

     trajectory = curve_dynamic_drawable(100); // number of steps stroed in the trajectory
     trajectory.uniform_parameter.color = {0,0,1};

     picked_object=-1;
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------//

    // setup initial position of the camera
    scene.camera.camera_type = camera_control_spherical_coordinates;
    scene.camera.scale = 10.0f;
    scene.camera.apply_rotation(0,0,0,1.2f);

    terrain = create_terrain();
    //terrain.uniform_parameter.color = vec3{0.8f,1.0f,0.8f};
    terrain.uniform_parameter.shading.specular = 0;
    texture_terrain = texture_gpu( image_load_png("data/terrain.png") );

    stade = create_corners_stade(5.0f, 3.9f, 4.0f, 0.0f, 0.0f);
    stade.uniform_parameter.color = {0.0f, 0.0f, 1.0f};

//--------------CREATING MESH OBJECTS FOR PLAYER-------------------------------------------//
    mesh_drawable body = mesh_primitive_cylinder(r_body, {0,delta,0}, {0,delta+l_body,0});
    body.uniform_parameter.color = {1.0f, 1.0f, 0.0f};
    mesh_drawable anche = mesh_primitive_cylinder(r_cylinder, {0,0,0}, {l_anche,0,0});
    mesh_drawable cylindre = mesh_primitive_cylinder(r_cylinder, {0,0,0}, {0,l_arm,0});
    cylindre.uniform_parameter.color = {0.737255f,0.560784f,0.560784f};
    mesh_drawable cylindre1 = mesh_primitive_cylinder(r_cylinder*1.2f, {0,0,0}, {0,l_arm,0});
    cylindre1.uniform_parameter.color = {0.0f,0.0f,1.0f};
    mesh_drawable cylindre_horiz = mesh_primitive_cylinder(r_cylinder, {-l_epo,0,0},{l_epo,0,0});
    cylindre_horiz.uniform_parameter.color = {1.0f, 1.0f, 0.0f};
    mesh_drawable cylindre_small = mesh_primitive_cylinder(r_cylinder, {0,0,0}, {0,l_arm/2,0});
    cylindre_small.uniform_parameter.color = {0.737255f,0.560784f,0.560784f};
    mesh_drawable cylindre_hand = mesh_primitive_cylinder(r_hand, {0,0,0}, {0,l_arm,0});
    cylindre_hand.uniform_parameter.color = {0.737255f,0.560784f,0.560784f};
    mesh_drawable sph = mesh_primitive_sphere(r_sph,{0,0,0},20,20);
    sph.uniform_parameter.color = {0.737255f,0.560784f,0.560784f};

    mesh_drawable pied = mesh_primitive_cylinder(r_pied, {0,0,0},{0,0,l_pied});
    pied.uniform_parameter.color = {0.0f,0.0f,0.0f};
    mesh_drawable head = mesh_primitive_sphere(r_head,{0,0,0},20,20);
    head.uniform_parameter.color = {0.737255f,0.560784f,0.560784f};
    mesh balon = mesh_primitive_sphere(r_balon*0.8f,{0,0,0},20,20);

//---------------CREATING HIERARCHY OF ALL PARTS OF BODY-----------------------------------//

    hierarchy.add_element(body, "body", "root");

    //-------------------------------UPPER BODY-----------------------------//
    hierarchy.add_element(cylindre_small, "neck", "body", {0, l_body+delta, 0});
    hierarchy.add_element(head, "head", "neck", {0, l_arm/2, 0});


    hierarchy.add_element(cylindre_horiz,"tors","body", {0,l_body+delta,0});
    hierarchy.add_element(sph, "ep1", "tors", {-l_epo,0,0});
    hierarchy.add_element(sph, "ep2", "tors", {l_epo,0,0});


    hierarchy.add_element(cylindre_hand, "lup1", "ep1",{-l_arm/10.0f,-l_arm,0});
    hierarchy.add_element(cylindre_hand, "lup2", "ep2",{l_arm/10.0f,-l_arm,0});

    hierarchy.add_element(sph, "ep3", "lup1",{0,0,0});
    hierarchy.add_element(sph, "ep4", "lup2",{0,0,0});

    hierarchy.add_element(cylindre_hand, "lup3", "ep3",{0,-l_arm,0});
    hierarchy.add_element(cylindre_hand, "lup4", "ep4",{0,-l_arm,0});

    //------------------------------DOWN BODY--------------------------------//
    hierarchy.add_element(anche, "a1", "body",{l_anche,delta,0});
    hierarchy.add_element(anche, "a2", "body",{-2.0f*l_anche,delta,0});

    hierarchy.add_element(sph, "sph1", "a1",{l_anche,0,0});
    hierarchy.add_element(sph, "sph2", "a2",{0,0,0});

    hierarchy.add_element(cylindre1, "l1", "sph1",{0,-l_arm,0});
    hierarchy.add_element(cylindre1, "l2", "sph2",{0,-l_arm,0});

    hierarchy.add_element(sph, "sph3", "l1",{0,0,0});
    hierarchy.add_element(sph, "sph4", "l2",{0,0,0});

    hierarchy.add_element(cylindre, "l3", "sph3",{0,-l_arm,0});
    hierarchy.add_element(cylindre, "l4", "sph4",{0,-l_arm,0});

    hierarchy.add_element(sph, "sph5", "l3",{0,0,0});
    hierarchy.add_element(sph, "sph6", "l4",{0,0,0});

    hierarchy.add_element(pied, "p1", "sph5",{0,-r_pied,-r_sph/2});
    hierarchy.add_element(pied, "p2", "sph6",{0,-r_pied,-r_sph/2});

    timer.scale = 1.0f;

    //----------------------------------------------------------------------------------------//
    supporteurs = create_supporteurs(5.0f, 3.9f, 4.0f, 0.0f, 0.0f);
    texture_supporteurs = texture_gpu( image_load_png("data/supporteursBrasil2.png") );


    //----------------------------------------------Sky---------------------------------------//
    skybox = create_skybox();
        skybox.uniform_parameter.shading = {1,0,0};
        skybox.uniform_parameter.rotation = rotation_from_axis_angle_mat3({1,0,0},-3.014f/2.0f);
        texture_skybox = texture_gpu(image_load_png("data/skybox.png"));

}



void scene_exercise::frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& )
{
    timer.update();

    //Movement de ballon
    const float t = timer.t;
    //

    //
    set_gui();

    glEnable( GL_POLYGON_OFFSET_FILL ); // avoids z-fighting when displaying wireframe
    glBindTexture(GL_TEXTURE_2D, texture_terrain);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

//---------------------------------------MOVEMENT OF THE BALL--------------------------------------------------------------

    const size_t idx = index_at_value(t, keyframe_time);

    const size_t N = keyframe_time.size();
    const float t1 = keyframe_time[idx];
    const float t2 = keyframe_time[idx+1];

    const vec3& p1 = keyframe_position[idx];
    const vec3& p2 = keyframe_position[idx+1];

    const vec3 p = linear_interpolation(t,t1,t2,p1,p2);
    trajectory.add_point(p);

    surface0.uniform_parameter.translation = p;
    surface0.draw(shaders["mesh"],scene.camera);

    const float amplitude = 0.5f;

//----------------------------------BODY MOVEMENT--------------------------------------------------------------------------

    hierarchy.rotation("body") = rotation_from_axis_angle_mat3({1,0,0}, amplitude/40*std::sin(1.5f*3.14f/timer.scale*(t))+3.14f/2);

    hierarchy.rotation("tors") = rotation_from_axis_angle_mat3({0,0,1}, amplitude/15*std::sin(2*3.14f/timer.scale*(t)));

    hierarchy.rotation("ep1") = rotation_from_axis_angle_mat3({0,0,1}, amplitude/10.f*std::sin(2*3.14f/timer.scale*(t))+3.25f/4);
    hierarchy.rotation("ep2") = rotation_from_axis_angle_mat3({0,0,1}, amplitude/20.f*std::sin(2*3.14f/timer.scale*(t))-3.14f/4);

    hierarchy.rotation("ep3") = rotation_from_axis_angle_mat3({1,0,0}, amplitude/10.f*std::sin(2*3.14f/timer.scale*(t))-3.25f/2);
    hierarchy.rotation("ep4") = rotation_from_axis_angle_mat3({1,0,0}, amplitude/20.f*std::sin(2*3.14f/timer.scale*(t))-3.14f/2);


    hierarchy.rotation("sph1") = rotation_from_axis_angle_mat3({1,0,0}, amplitude/2.5f*std::sin(1.5f*3.1415f/timer.scale*(t)));
    hierarchy.rotation("sph3") = rotation_from_axis_angle_mat3({1,0,0}, amplitude/1.5f*std::sin(1.5f*3.1415f/timer.scale*(t)));

    hierarchy.rotation("sph2") = rotation_from_axis_angle_mat3({1,0,0}, amplitude/2.5f*std::sin(1.5f*3.1415f/timer.scale*(t+1.0f/1.5f*3.14f)));
    hierarchy.rotation("sph4") = rotation_from_axis_angle_mat3({1,0,0}, amplitude/1.5f*std::sin(1.5f*3.1415f/timer.scale*(t+1.0f/1.5f*3.14f)));

    hierarchy.draw(shaders["mesh"], scene.camera);

//----------------------------------DISPLAY ALL PARTS OF SCENE--------------------------------------------------------------

    // Display terrain
    glPolygonOffset( 1.0, 1.0 );
    terrain.draw(shaders["mesh"], scene.camera);

    if( gui_scene.wireframe ){
        glPolygonOffset( 1.0, 1.0 );
        terrain.draw(shaders["mesh"], scene.camera);
    }

    // Display stade
    glPolygonOffset( 1.0, 1.0 );
    stade.draw(shaders["mesh"], scene.camera);

    if( gui_scene.wireframe ){
        glPolygonOffset( 1.0, 1.0 );
        stade.draw(shaders["mesh"], scene.camera);
        //ADDED STUFF
        hierarchy.draw(shaders["mesh"], scene.camera);
        //
    }


     glBindTexture(GL_TEXTURE_2D, texture_supporteurs);

     glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
     glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    // Display supporteurs
    glPolygonOffset( 1.0, 1.0 );
    supporteurs.draw(shaders["mesh"], scene.camera);

    if( gui_scene.wireframe ){
    glPolygonOffset( 1.0, 1.0 );
    supporteurs.draw(shaders["wireframe"], scene.camera);
    }


    display_skybox(shaders, scene);
}

void scene_exercise::display_skybox(std::map<std::string,GLuint>& shaders, scene_structure& scene)
{
    if(gui_scene.skybox)
    {
        if(gui_scene.texture)
            glBindTexture(GL_TEXTURE_2D,texture_skybox);
        skybox.uniform_parameter.scaling = 150.0f;
        skybox.uniform_parameter.translation = scene.camera.camera_position() + vec3(0,0,-50.0f);
        skybox.draw(shaders["mesh"], scene.camera);
        glBindTexture(GL_TEXTURE_2D,scene.texture_white);
    }
}


vcl::mesh create_skybox()
{
    const vec3 p000 = {-1,-1,-1};
    const vec3 p001 = {-1,-1, 1};
    const vec3 p010 = {-1, 1,-1};
    const vec3 p011 = {-1, 1, 1};
    const vec3 p100 = { 1,-1,-1};
    const vec3 p101 = { 1,-1, 1};
    const vec3 p110 = { 1, 1,-1};
    const vec3 p111 = { 1, 1, 1};

    mesh skybox;

    skybox.position = {
        p000, p100, p110, p010,
        p010, p110, p111, p011,
        p100, p110, p111, p101,
        p000, p001, p010, p011,
        p001, p101, p111, p011,
        p000, p100, p101, p001
    };


    skybox.connectivity = {
        {0,1,2}, {0,2,3}, {4,5,6}, {4,6,7},
        {8,11,10}, {8,10,9}, {17,16,19}, {17,19,18},
        {23,22,21}, {23,21,20}, {13,12,14}, {13,14,15}
    };

    const float e = 1e-3f;
    const float u0 = 0.0f;
    const float u1 = 0.25f+e;
    const float u2 = 0.5f-e;
    const float u3 = 0.75f-e;
    const float u4 = 1.0f;
    const float v0 = 0.0f;
    const float v1 = 1.0f/3.0f+e;
    const float v2 = 2.0f/3.0f-e;
    const float v3 = 1.0f;
    skybox.texture_uv = {
        {u1,v1}, {u2,v1}, {u2,v2}, {u1,v2},
        {u1,v2}, {u2,v2}, {u2,v3}, {u1,v3},
        {u2,v1}, {u2,v2}, {u3,v2}, {u3,v1},
        {u1,v1}, {u0,v1}, {u1,v2}, {u0,v2},
        {u4,v1}, {u3,v1}, {u3,v2}, {u4,v2},
        {u1,v1}, {u2,v1}, {u2,v0}, {u1,v0}
    };


    return skybox;

}




float evaluate_terrain_z(float u, float v)
{
    return 0;
}


vec3 evaluate_terrain(float u, float v)
{
    const float x = 20*(u-0.5f);
    const float y = 20*(v-0.5f);
    const float z = evaluate_terrain_z(u,v);

    return {x,y,z};
}


mesh create_terrain()
{
    // Number of samples of the terrain is N x N
    const size_t N = 100;

    mesh terrain; // temporary terrain storage (CPU only)
    terrain.position.resize(N*N);
    terrain.texture_uv.resize(N*N);
    // Fill terrain geometry
    for(size_t ku=0; ku<N; ++ku)
    {
        for(size_t kv=0; kv<N; ++kv)
        {
            // Compute local parametric coordinates (u,v) \in [0,1]
            const float u = ku/(N-1.0f);
            const float v = kv/(N-1.0f);

            // Compute coordinates
            terrain.position[kv+N*ku] = evaluate_terrain(u,v);
            terrain.texture_uv[kv+N*ku] = {u,v};
        }
    }


    // Generate triangle organization
    //  Parametric surface with uniform grid sampling: generate 2 triangles for each grid cell
    for(size_t ku=0; ku<N-1; ++ku)
    {
        for(size_t kv=0; kv<N-1; ++kv)
        {
            const unsigned int Ns = N;
            const unsigned int idx = kv + Ns*ku; // current vertex offset

            const index3 triangle_1 = {idx, idx+1+Ns, idx+1};
            const index3 triangle_2 = {idx, idx+Ns, idx+1+Ns};

            terrain.connectivity.push_back(triangle_1);
            terrain.connectivity.push_back(triangle_2);
        }
    }

    return terrain;
}


mesh create_corners_stade(float scale, float smallSide, float bigSide, float x_centre, float y_centre)
{
    mesh m;
    float x_reflection = 1.0f + bigSide / 2.0f;
    float y_reflection = 1.0f + smallSide / 2.0f;
    vec3 translation = {x_centre - x_reflection*scale, y_centre - y_reflection*scale, 0.0f };
    //corners
    //first corner
    const vec3 p11 = {1.0f, 0.0f, 1.0f};
    const vec3 p12 = {1.0f, 0.0f, 0.0f};
    const vec3 p13 = {1.0f, 1.0f, 0.0f};
    const vec3 p14 = {0.0f, 1.0f, 0.0f};
    const vec3 p15 = {0.0f, 1.0f, 1.0f};
    m.position.push_back( p11 *scale+translation);
    m.position.push_back( p12 *scale+translation );
    m.position.push_back( p13 *scale+translation);
    m.position.push_back( p14 *scale+translation);
    m.position.push_back( p15 *scale+translation);
    //2nd corner == y_reflection of first corner
    const vec3 p21 = {1.0f, 2*y_reflection - 0.0f, 1.0f};
    const vec3 p22 = {1.0f, 2*y_reflection - 0.0f, 0.0f};
    const vec3 p23 = {1.0f, 2*y_reflection - 1.0f, 0.0f};
    const vec3 p24 = {0.0f, 2*y_reflection - 1.0f, 0.0f};
    const vec3 p25 = {0.0f, 2*y_reflection - 1.0f, 1.0f};
    m.position.push_back( p21 *scale+translation);
    m.position.push_back( p22 *scale+translation);
    m.position.push_back( p23 *scale+translation);
    m.position.push_back( p24 *scale+translation);
    m.position.push_back( p25 *scale+translation);
    //third corner == x_reflection of 2nd corner
    const vec3 p31 = {2*x_reflection - 1.0f, 2*y_reflection - 0.0f, 1.0f};
    const vec3 p32 = {2*x_reflection - 1.0f, 2*y_reflection - 0.0f, 0.0f};
    const vec3 p33 = {2*x_reflection - 1.0f, 2*y_reflection - 1.0f, 0.0f};
    const vec3 p34 = {2*x_reflection - 0.0f, 2*y_reflection - 1.0f, 0.0f};
    const vec3 p35 = {2*x_reflection - 0.0f, 2*y_reflection - 1.0f, 1.0f};
    m.position.push_back( p31 *scale+translation);
    m.position.push_back( p32 *scale+translation);
    m.position.push_back( p33 *scale+translation);
    m.position.push_back( p34 *scale+translation);
    m.position.push_back( p35 *scale+translation);
    //fourth corner == x_reflection of first corner
    const vec3 p41 = {2*x_reflection - 1.0f, 0.0f, 1.0f};
    const vec3 p42 = {2*x_reflection - 1.0f, 0.0f, 0.0f};
    const vec3 p43 = {2*x_reflection - 1.0f, 1.0f, 0.0f};
    const vec3 p44 = {2*x_reflection - 0.0f, 1.0f, 0.0f};
    const vec3 p45 = {2*x_reflection - 0.0f, 1.0f, 1.0f};
    m.position.push_back( p41 *scale+translation);
    m.position.push_back( p42 *scale+translation);
    m.position.push_back( p43 *scale+translation);
    m.position.push_back( p44 *scale+translation);
    m.position.push_back( p45 *scale+translation);


    // Connectivity
    for(size_t k=0; k<20; k+=5)
    {
    const unsigned int u1 = k;
    const unsigned int u2 = (k+1);
    const unsigned int u3 = (k+2);
    const unsigned int u4 = (k+3);
    const unsigned int u5 = (k+4);


    const index3 t1 = {u1, u2, u3};
    const index3 t2 = {u3, u4, u5};
    const index3 t3 = {u1, u3, u5};
    const index3 t4 = {u1, u3, u5};
    const index3 t5 = {u1, u2, u4};
    const index3 t6 = {u1, u4, u5};

    //creating corners
    m.connectivity.push_back(t1);
    m.connectivity.push_back(t2);
    m.connectivity.push_back(t3);
    m.connectivity.push_back(t4);
    m.connectivity.push_back(t5);
    m.connectivity.push_back(t6);
    }

    //create sides
    const index3 t13 = { 4,3, 8};
    const index3 t14 = { 8,4, 9};
    m.connectivity.push_back(t13);
    m.connectivity.push_back(t14);

    const index3 t23 = { 6,5, 10};
    const index3 t24 = { 10,6, 11};
    m.connectivity.push_back(t23);
    m.connectivity.push_back(t24);

    const index3 t33 = { 14,13, 19};
    const index3 t34 = { 18,13, 19};
    m.connectivity.push_back(t33);
    m.connectivity.push_back(t34);

    const index3 t43 = { 16,15, 1};
    const index3 t44 = { 0,15, 1};
    m.connectivity.push_back(t43);
    m.connectivity.push_back(t44);


    //couverture
    //Geometry
    const vec3 pC1 = {1.0f, 1.0f, 2.0f};
    const vec3 pC2 = {1.0f, 2*y_reflection - 1.0f, 2.0f};
    const vec3 pC3 = {2*x_reflection - 1.0f, 2*y_reflection - 1.0f, 2.0f};
    const vec3 pC4 = {2*x_reflection - 1.0f, 1.0f, 2.0f};
    m.position.push_back( pC1 *scale+translation);//20
    m.position.push_back( pC2 *scale+translation);//21
    m.position.push_back( pC3 *scale+translation);//22
    m.position.push_back( pC4 *scale+translation);//23
    //Connectivity
    const index3 C11 = { 0,20, 4};
    const index3 C13 = { 14,22, 10};
    //couverture sides
    const index3 C21 = { 4,20, 9};
    const index3 C23 = { 5,22, 10};
    const index3 C25 = { 19,22, 14};
    const index3 C28 = { 0,15, 20};

    m.connectivity.push_back(C11);
    m.connectivity.push_back(C13);
    m.connectivity.push_back(C21);
    m.connectivity.push_back(C23);
    m.connectivity.push_back(C25);
    m.connectivity.push_back(C28);

    return m;
}

mesh create_supporteurs(float scale, float smallSide, float bigSide, float x_centre, float y_centre)
{
 mesh m;
 float x_reflection = 1.0f + bigSide / 2.0f;
 float y_reflection = 1.0f + smallSide / 2.0f;
 vec3 translation = {x_centre - x_reflection*scale, y_centre - y_reflection*scale, 0.0f };
 //corners
 //first corner
 const vec3 p13 = {1.0f, 1.0f, 0.0f};//2
 m.position.push_back( p13 *scale+translation);
 const vec3 p15 = {0.0f, 1.0f, 1.0f};//4
 m.position.push_back( p15 *scale+translation);
 const vec3 aux3 = {1.0f, 2*y_reflection - 1.0f, 0.0f};//7
 m.position.push_back( aux3 *scale+translation);
 const vec3 p25 = {0.0f, 2*y_reflection - 1.0f, 1.0f};//9
 m.position.push_back( p25 *scale+translation);
 //2nd corner == y_reflection of first corner

 const vec3 p21 = {1.0f, 2*y_reflection - 0.0f, 1.0f};//5
 m.position.push_back( p21 *scale+translation);
 const vec3 p23 = {1.0f, 2*y_reflection - 1.0f, 0.0f};//7
 m.position.push_back( p23 *scale+translation);
 const vec3 p31 = {2*x_reflection - 1.0f, 2*y_reflection - 0.0f, 1.0f};//10
 m.position.push_back( p31 *scale+translation);
 const vec3 aux2 = {2*x_reflection - 1.0f, 2*y_reflection - 1.0f, 0.0f};//12
 m.position.push_back( aux2 *scale+translation);
 //third corner == x_reflection of 2nd corner
 const vec3 p33 = {2*x_reflection - 1.0f, 2*y_reflection - 1.0f, 0.0f};//12
 m.position.push_back( p33 *scale+translation);
 const vec3 p35 = {2*x_reflection - 0.0f, 2*y_reflection - 1.0f, 1.0f};//14
 m.position.push_back( p35 *scale+translation);
 const vec3 aux4 = {2*x_reflection - 1.0f, 1.0f, 0.0f};//17
 m.position.push_back( aux4 *scale+translation);
 const vec3 p45 = {2*x_reflection - 0.0f, 1.0f, 1.0f};//19
 m.position.push_back( p45 *scale+translation);
 //fourth corner == x_reflection of first corner
 const vec3 p41 = {2*x_reflection - 1.0f, 0.0f, 1.0f};//15
 m.position.push_back( p41 *scale+translation);
 const vec3 p43 = {2*x_reflection - 1.0f, 1.0f, 0.0f};//17
 m.position.push_back( p43 *scale+translation);
 const vec3 p11 = {1.0f, 0.0f, 1.0f};//0
 m.position.push_back( p11 *scale+translation);
 const vec3 aux1 = {1.0f, 1.0f, 0.0f};//2
 m.position.push_back( aux1 *scale+translation);



 //create sides
 const index3 t13 = {1,0,2};
 const index3 t14 = {1,2,3};
 m.connectivity.push_back(t13);
 m.connectivity.push_back(t14);

 const index3 t23 = { 5,4, 6};
 const index3 t24 = { 5,6, 7};
 m.connectivity.push_back(t23);
 m.connectivity.push_back(t24);

 const index3 t33 = { 9,8, 10};
 const index3 t34 = { 9,10, 11};
 m.connectivity.push_back(t33);
 m.connectivity.push_back(t34);

 const index3 t43 = { 13,12, 14};
 const index3 t44 = { 13,14, 15};
 m.connectivity.push_back(t43);
 m.connectivity.push_back(t44);

 m.texture_uv = {{0,0}, {0,1}, {1,1}, {1,0},{0,0}, {0,1}, {1,1}, {1,0},{0,0}, {0,1}, {1,1}, {1,0},{0,0}, {0,1}, {1,1}, {1,0}};

 return m;
}

void scene_exercise::set_gui()
{
    ImGui::Checkbox("Wireframe", &gui_scene.wireframe);

    ImGui::Checkbox("Skybox", &gui_scene.skybox);
}





#endif
