#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <learnopengl/shader.h>
#include <learnopengl/camera.h>
#include <learnopengl/model.h>

#include "urdf_parser/urdf_parser.h"
#include <iostream>
#include <fstream>

#include <unistd.h>

using namespace urdf;
using namespace std;

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void renderScene(const Shader &shader, bool move);
void processInput(GLFWwindow *window);

// settings
const unsigned int SCR_WIDTH = 1280;
const unsigned int SCR_HEIGHT = 720;
bool replayKeyPressed = false;
bool shadows = true;
bool shadowsKeyPressed = false;
float light_color = 0.6;
int MAX_TICK = 1;
int tick = 0;
int c = 0;

// camera
Camera camera(glm::vec3(1.0f, 1.0f, 3.0f));
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;

// file name
const string delimiter1 = "://fetch_description/";
const string delimiter2 = "../../resources/fetch_description/";
const string urdf_file_path = "../../resources/fetch_description/urdf/fetch.urdf";
string config_file_path = "../../resources/configs/fetch_say_hi";

// timing
float deltaTime = 0.0f;
float lastFrame = 0.0f;

// configurations file
vector<vector<float>> configs;
vector<shared_ptr<Model_>> meshes;
vector<glm::mat4> initTransMat;
const glm::mat4 init = glm::mat4(1.0f);

glm::mat4 calTransMat (shared_ptr<Model_> link)
{    
    if (link->parent == nullptr)
        return link->joint_transmat;
        
    return calTransMat(link->parent) * link->joint_transmat;
}

void addChildLinks(LinkConstSharedPtr link, shared_ptr<Model_> parent)
{
    double roll, pitch, yaw;
    double x, y, z;
    double roll_, pitch_, yaw_;
    double x_, y_, z_;
    double axis_x, axis_y, axis_z = 0.0f;
    GeometrySharedPtr geom;
    string token;
    string file_name;
    shared_ptr<Model_> child_node;
    
    
    for (vector<LinkSharedPtr>::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++)
    {
        (*child)->parent_joint->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch, yaw); // rotation
        x = (*child)->parent_joint->parent_to_joint_origin_transform.position.x;
        y = (*child)->parent_joint->parent_to_joint_origin_transform.position.y;
        z = (*child)->parent_joint->parent_to_joint_origin_transform.position.z;
        // cout << (*child)->name << endl;  
        // cout << "xyz: " << x << y << z << endl;

        // UNKNOWN 0  REVOLUTE 1  PRISMATIC 2  FLOATING 3  PLANAR 4  FIXED 5
        if ((*child)->parent_joint->type) // only revolute joint available for now
        { 
            axis_x = (*child)->parent_joint->axis.x;
            axis_y = (*child)->parent_joint->axis.y;
            axis_z = (*child)->parent_joint->axis.z;
        }            
        glm::vec3 axis = glm::vec3(axis_x,axis_y,axis_z);
        
        if ((*child)->visual)
        {
            geom = (*child)->visual->geometry;
            glm::mat4 joint_transmat = init;
            joint_transmat = glm::translate(joint_transmat, glm::vec3(x,y,z)); 
            joint_transmat = glm::rotate(joint_transmat, float(roll), glm::vec3(-1.,0.,0.)); 
            joint_transmat = glm::rotate(joint_transmat, float(pitch),  glm::vec3(0.,-1.,0.)); 
            joint_transmat = glm::rotate(joint_transmat, float(yaw), glm::vec3(0.,0.,-1.));     

            glm::mat4 vis_transmat = init;
            (*child)->visual->origin.rotation.getRPY(roll_, pitch_, yaw_); // rotation
            x_ = (*child)->visual->origin.position.x;
            y_ = (*child)->visual->origin.position.y;
            z_ = (*child)->visual->origin.position.z;
            vis_transmat = glm::translate(vis_transmat, glm::vec3(x_,y_,z_)); 
            vis_transmat = glm::rotate(vis_transmat, float(roll_), glm::vec3(1.,0.,0.)); 
            vis_transmat = glm::rotate(vis_transmat, float(pitch_),  glm::vec3(0.,1.,0.)); 
            vis_transmat = glm::rotate(vis_transmat, float(yaw_), glm::vec3(0.,0.,1.));   

            string mat_name;
            
            if (geom->type == MESH)
            {   
                MeshSharedPtr m = urdf::dynamic_pointer_cast<urdf::Mesh>(geom);
                string token = m->filename.substr(m->filename.find(delimiter1)+delimiter1.length(), m->filename.length());
                file_name = delimiter2 + token;

                // material
                if ((*child)->visual->material != nullptr)
                    mat_name = (*child)->visual->material->name;
                else
                    mat_name == "White";

                glm::vec3 scale = glm::vec3(m->scale.x, m->scale.y, m->scale.z);
                shared_ptr<Model_> child_node(new Model_ ((*child)->name, file_name, mat_name, joint_transmat, vis_transmat, scale, axis, parent));
                if (mat_name == "" && (*child)->visual->material != nullptr)  child_node->color = glm::vec3((*child)->visual->material->color.r, (*child)->visual->material->color.g,(*child)->visual->material->color.b);
                
                meshes.push_back(child_node);
                addChildLinks(*child, child_node); 
            }        
            else if (geom->type == SPHERE) 
            {  
                SphereSharedPtr m = urdf::dynamic_pointer_cast<Sphere>(geom);
                string file_name = "../../resources/objects/sphere.stl"; // HACKING !!!
                
                glm::vec3 scale = glm::vec3(m->radius, m->radius, m->radius);
                shared_ptr<Model_> child_node(new Model_ ((*child)->name, file_name, mat_name,  joint_transmat, vis_transmat, scale, axis, parent));
                if (mat_name == "" && (*child)->visual->material != nullptr)  child_node->color = glm::vec3((*child)->visual->material->color.r, (*child)->visual->material->color.g,(*child)->visual->material->color.b);

                meshes.push_back(child_node);
                addChildLinks(*child, child_node); 
            }
            else if (geom->type == BOX)
            {
                BoxSharedPtr m = urdf::dynamic_pointer_cast<Box>(geom);
                string file_name = "../../resources/objects/box.stl"; // HACKING !!!
                
                glm::vec3 scale = glm::vec3(m->dim.x, m->dim.y, m->dim.z);
                shared_ptr<Model_> child_node(new Model_ ((*child)->name, file_name, mat_name,  joint_transmat, vis_transmat, scale, axis, parent));
                if (mat_name == "" && (*child)->visual->material != nullptr)  child_node->color = glm::vec3((*child)->visual->material->color.r, (*child)->visual->material->color.g,(*child)->visual->material->color.b);

                meshes.push_back(child_node);
                addChildLinks(*child, child_node); 
            }
            else if (geom->type == CYLINDER)
            {
                CylinderSharedPtr m = urdf::dynamic_pointer_cast<Cylinder>(geom);
                string file_name = "../../resources/objects/cylinder.stl"; // HACKING !!!
                
                glm::vec3 scale = glm::vec3(m->radius, m->radius, m->length);
                shared_ptr<Model_> child_node(new Model_ ((*child)->name, file_name, mat_name,  joint_transmat, vis_transmat, scale, axis, parent));
                if (mat_name == "" && (*child)->visual->material != nullptr)  child_node->color = glm::vec3((*child)->visual->material->color.r, (*child)->visual->material->color.g,(*child)->visual->material->color.b);

                meshes.push_back(child_node);
                addChildLinks(*child, child_node); 
            }        
        }
    }
}

int main(int argc, char** argv)
{    
    // urdf: read urdf file
    // -------------------------
    string xml_string;
    fstream xml_file(urdf_file_path, fstream::in);
    while ( xml_file.good() )
    {
        string line;
        getline( xml_file, line);
        xml_string += (line + "\n");
    }
    xml_file.close();
    ModelInterfaceSharedPtr robot = parseURDF(xml_string);
    if (!robot){
        cerr << "ERROR: Model Parsing the xml failed" << endl;
        return -1;
    }
    string output = robot->getName();  
    
    // read configs file
    // -------------------------    
    ifstream config_file(config_file_path.data());
    if (config_file.is_open()){
        string line;
        int i = 0;
        int len = 0;
        vector<float> config;
        while (getline(config_file, line)){
            // length of the config in the very first line of the text file
            if (i == 0) len = stof(line);   
            else {
                if (i % len != 0)
                    config.push_back(stof(line));
                else {
                    config.push_back(stof(line));
                    configs.push_back(config);
                    config.clear();
                }
            }
            i++;
        }
        config_file.close();
    }
  
    // glfw: initialize and configure
    // ------------------------------
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    
    // glfw window creation
    // --------------------
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "glURDF", NULL, NULL);
    if (window == NULL)
    {
        cout << "Failed to create GLFW window" << endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);

    // tell GLFW to capture our mouse
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);


    // glad: load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        cout << "Failed to initialize GLAD" << endl;
        return -1;
    }

    // configure global opengl state
    // -----------------------------
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    // build and compile shaders
    // -------------------------
    Shader shader("point_shadows.vs", "point_shadows.fs");
    Shader simpleDepthShader("point_shadows_depth.vs", "point_shadows_depth.fs", "point_shadows_depth.gs");

    
    // configure depth map FBO
    // -----------------------
    const unsigned int SHADOW_WIDTH = 1024, SHADOW_HEIGHT = 1024;
    unsigned int depthMapFBO;
    glGenFramebuffers(1, &depthMapFBO);
    // create depth cubemap texture
    unsigned int depthCubemap;
    glGenTextures(1, &depthCubemap);
    glBindTexture(GL_TEXTURE_CUBE_MAP, depthCubemap);
    for (unsigned int i = 0; i < 6; ++i)
        glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_DEPTH_COMPONENT, SHADOW_WIDTH, SHADOW_HEIGHT, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    // attach depth texture as FBO's depth buffer
    glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
    glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, depthCubemap, 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // shader configuration
    // --------------------
    shader.use();
    shader.setInt("diffuseTexture", 0);
    shader.setInt("depthMap", 1);

    // lighting info
    // -------------
    glm::vec3 lightPos(5.0f, 5.0f, 5.0f);

    // load robot model
    // -----------    
    LinkConstSharedPtr link = robot->getRoot();
    GeometrySharedPtr geom = link->visual->geometry;
    MeshSharedPtr m = urdf::dynamic_pointer_cast<Mesh>(geom);
    std::string token = m->filename.substr(m->filename.find(delimiter1)+delimiter1.length(), m->filename.length());
    std::string file_name = delimiter2 + token;
    glm::vec3 scale = glm::vec3(1.0f); glm::vec3 axis = glm::vec3(0.0f);
    
    shared_ptr<Model_> root_node(new Model_ (link->name, file_name, link->visual->material->name, init, init, scale, axis, nullptr));
    meshes.push_back(root_node);
    addChildLinks(link, root_node);
    
    // draw in wireframe    
    //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    
    //init robot pose and scene
    for (int i = 0; i < meshes.size(); i++)
    {    
        if (i == 0)
        {
            // init pose
            glm::mat4 tmp = glm::rotate(init, glm::radians(90.0f), glm::vec3(-1.0f,0.0f,0.0f));  
            tmp = glm::rotate(tmp, glm::radians(90.0f), glm::vec3(0.0f,0.0f,-1.0f));  
            meshes[i]->joint_transmat = tmp;
        }        
        initTransMat.push_back(meshes[i]->joint_transmat);
    }    
    
    tick = 0;
    c = 0;

    cout << meshes.size() << endl;

    // render loop
    // -----------
    while (!glfwWindowShouldClose(window))
    {
        tick ++; 
        // per-frame time logic
        // --------------------
        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // input
        // -----
        processInput(window);

        // render
        // ------
        glClearColor(0.15f, 0.15f, 0.15f, 1.0f);
        //glClearColor(0.5f,0.8f,0.9f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        // create depth cubemap transformation matrices
        // -----------------------------------------------
        float near_plane = 1.0f;
        float far_plane = 25.0f;
        glm::mat4 shadowProj = glm::perspective(glm::radians(90.0f), (float)SHADOW_WIDTH / (float)SHADOW_HEIGHT, near_plane, far_plane);
        std::vector<glm::mat4> shadowTransforms;
        shadowTransforms.push_back(shadowProj * glm::lookAt(lightPos, lightPos + glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f)));
        shadowTransforms.push_back(shadowProj * glm::lookAt(lightPos, lightPos + glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f)));
        shadowTransforms.push_back(shadowProj * glm::lookAt(lightPos, lightPos + glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f)));
        shadowTransforms.push_back(shadowProj * glm::lookAt(lightPos, lightPos + glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f, 0.0f, -1.0f)));
        shadowTransforms.push_back(shadowProj * glm::lookAt(lightPos, lightPos + glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f, -1.0f, 0.0f)));
        shadowTransforms.push_back(shadowProj * glm::lookAt(lightPos, lightPos + glm::vec3(0.0f, 0.0f, -1.0f), glm::vec3(0.0f, -1.0f, 0.0f)));

        // render scene to depth cubemap
        // --------------------------------
        glViewport(0, 0, SHADOW_WIDTH, SHADOW_HEIGHT);
        glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
        glClear(GL_DEPTH_BUFFER_BIT);
        simpleDepthShader.use();
        for (unsigned int i = 0; i < 6; ++i)
            simpleDepthShader.setMat4("shadowMatrices[" + std::to_string(i) + "]", shadowTransforms[i]);
        simpleDepthShader.setFloat("far_plane", far_plane);
        simpleDepthShader.setVec3("lightPos", lightPos);
        renderScene(simpleDepthShader, true);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        // render scene as normal 
        // -------------------------
        glViewport(0, 0, SCR_WIDTH, SCR_HEIGHT);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        shader.use();
        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
        glm::mat4 view = camera.GetViewMatrix();
        shader.setMat4("projection", projection);
        shader.setMat4("view", view);
        // set lighting uniforms
        shader.setVec3("lightPos", lightPos);
        shader.setVec3("viewPos", camera.Position);
        shader.setInt("shadows", shadows); // enable/disable shadows by pressing 'SPACE'
        shader.setFloat("far_plane", far_plane);
        shader.setVec3("lightColor", glm::vec3(light_color));
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_CUBE_MAP, depthCubemap);
        renderScene(shader, false);

        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        // -------------------------------------------------------------------------------
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    

    // glfw: terminate, clearing all previously allocated GLFW resources.
    // ------------------------------------------------------------------
    glfwTerminate();
    return 0;
}

void renderScene(const Shader &shader, bool move)
{
    glEnable(GL_CULL_FACE);
    
    // render robot
    int cnt = 0; // HACKING!!
    for (int i = 0; i < meshes.size(); i++)
    {          
        if (tick % MAX_TICK == 0 && c < configs.size())
        {
            if (meshes[i]->axis != glm::vec3(0.0f))
            {   
                meshes[i]->joint_transmat = glm::rotate(initTransMat[i], configs[c][cnt], meshes[i]->axis);  
                cnt++;
            }
        }
        glm::mat4 transmat = init;
        transmat = glm::scale(transmat, meshes[i]->scale);
        transmat = meshes[i]->vis_transmat * transmat;

        for (int j = 0; j < 3; j++)
        {
            if (meshes[i]->scale[j] < 0)
                glDisable(GL_CULL_FACE);
        }

        if (!meshes[i]->textures_loaded.empty())
            shader.setInt("textures", 1);
        else
            shader.setInt("textures", 0);
        
        transmat = calTransMat(meshes[i]) * transmat;
        shader.setVec3("color", meshes[i]->color);
        shader.setMat4("model", transmat);
        meshes[i]->Draw(shader);
    }
    
    if (tick % MAX_TICK == 0 && move) c++;
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow *window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    // camera pos
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.ProcessKeyboard(FORWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboard(BACKWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboard(LEFT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboard(RIGHT, deltaTime);
        
    // light color
    if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS){
        if (light_color < 1.0)
            light_color += 0.05;
    }
    if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS){
        if (light_color > 0.0)
            light_color -= 0.05;
    }
    
    // replay the motion
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS && !replayKeyPressed)
    {
        replayKeyPressed = true;
        tick = 0; c= 0;
    }
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_RELEASE)
    {
        replayKeyPressed = false;
    }
    
    // shadow on/off
    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS && !shadowsKeyPressed)
    {
        shadows = !shadows;
        shadowsKeyPressed = true;
    }
    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_RELEASE)
    {
        shadowsKeyPressed = false;
    }
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and 
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}

// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------
void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
    if (firstMouse)
    {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

    lastX = xpos;
    lastY = ypos;

    camera.ProcessMouseMovement(xoffset, yoffset);
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    camera.ProcessMouseScroll(yoffset);
}
