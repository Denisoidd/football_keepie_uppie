#pragma once

#include "../../exercises/base_exercise/base_exercise.hpp"

#ifdef INF443_INTRODUCTION


//struct scene_exercise : base_scene_exercise
//{

//    /** A part must define two functions that are called from the main function:
//     * setup_data: called once to setup data before starting the animation loop
//     * frame_draw: called at every displayed frame within the animation loop
//     *
//     * These two functions receive the following parameters
//     * - shaders: A set of shaders.
//     * - scene: Contains general common object to define the 3D scene. Contains in particular the camera.
//     * - data: The part-specific data structure defined previously
//     * - gui: The GUI structure allowing to create/display buttons to interact with the scene.
//    */

//    void setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
//    void frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);


//    // visual representation of a surface
//    vcl::mesh_drawable surface;
//    vcl::mesh_drawable sphere;
//};

// Stores some parameters that can be set from the GUI
struct gui_scene_structure
{
    bool wireframe = true;
    bool skybox = true;
    bool texture = true;
};

struct scene_exercise : base_scene_exercise
{

    /** A part must define two functions that are called from the main function:
     * setup_data: called once to setup data before starting the animation loop
     * frame_draw: called at every displayed frame within the animation loop
     *
     * These two functions receive the following parameters
     * - shaders: A set of shaders.
     * - scene: Contains general common object to define the 3D scene. Contains in particular the camera.
     * - data: The part-specific data structure defined previously
     * - gui: The GUI structure allowing to create/display buttons to interact with the scene.
    */

    void setup_data(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);
    void frame_draw(std::map<std::string,GLuint>& shaders, scene_structure& scene, gui_structure& gui);

    void set_gui();

    //Movement de ballon-------------------------------------------------------------
    // Data (p_i,t_i)
    std::vector<vcl::vec3> keyframe_position; // Given positions
    std::vector<float> keyframe_time;         // Time at given positions

    vcl::mesh_drawable surface0;                            // moving point
    vcl::mesh_drawable sphere0;                             // keyframe samples
    vcl::segment_drawable_immediate_mode segment_drawer;   // used to draw segments between keyframe samples
    vcl::curve_dynamic_drawable trajectory;                // Draw the trajectory of the moving point as a curve

    // Store the index of a selected sphere
    int picked_object;
    //-------------------------------------------------------------------------------

    // visual representation of a surface
    vcl::mesh_drawable terrain;
    vcl::mesh_drawable sphere;

    vcl::mesh_drawable stade;
    vcl::mesh_drawable supporteurs;
    vcl::mesh_drawable tree_foliage;
    std::vector<vcl::vec3> tree_position;
    void update_tree_position();
    void display_tree(std::map<std::string,GLuint>& shaders, scene_structure& scene);


    vcl::mesh_drawable mushroom_trunc;
    vcl::mesh_drawable mushroom_top;
    std::vector<vcl::vec3> mushroom_position;
    void update_mushroom_position();
    void display_mushroom(std::map<std::string,GLuint>& shaders, scene_structure& scene);


    vcl::mesh_drawable skybox;
    GLuint texture_skybox;
    void display_skybox(std::map<std::string,GLuint>& shaders, scene_structure& scene);


    GLuint texture_terrain;
    GLuint texture_player;

    GLuint texture_supporteurs;
    //-----------ADDED STUFF----------------
    vcl::mesh_drawable surface;
    GLuint texture_id;

    vcl::mesh_drawable_hierarchy hierarchy;
    vcl::mesh_drawable ground;
    //--------------------------------------

    gui_scene_structure gui_scene;

    //--------------------------------------
    vcl::timer_interval timer;
    //--------------------------------------
};

#endif
