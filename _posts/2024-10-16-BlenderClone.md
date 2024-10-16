---
layout: post
title: "Project Showcase - Blender-like 3D graphics software in OpenGL"
date: 2024-10-2 10:00:00 +0300
categories: [Computer Graphics, Rasterization & Ray Tracing]
tags: [computer graphics]
image: ss.png
math: true
img_path: /assets/images/BlenderCloneImages
---

## Introduction

A few years ago I was really attracted by the idea of writing my own ray tracer. There are some really good resources on writing ray tracers from bare-bones to almost industry-level.

Fast-forward a bit and after a few articles I started working on it. But I was quite frustrated with the slow and unoptimized render times. I hated how I had absolutely no preview of the scenes before ray tracing and had to place meshes, cameras and lights pretty much blind.

So I paused the development of my own hobby tracer to build the basis of a new project that would eventually incorporate my ray tracer: a Blender-like 3D graphics program. Some features I wanted from the start were:
1. Mesh imports
2. PBR Rasterization
3. Final render with ray tracing
4. Seamless integration of rasterizer with ray tracer

And while the first two are not incredibly hard, the last two are the hard stuff. Building a unified architecture that would support both rasterization in OpenGL *and* custom CPU/GPU ray tracing was something that was way above my skill level and I knew that from the start. Retroactive refactoring of my terriblr code is inevitable. Doesn't stop me from trying though...

## Dependencies
- OpenGL w/ GLAD & GLFW
- spdlog
- Dear ImGUI - docking branch
- ImGuizmo
- stb_image.h
- Assimp library

## Entry Point, Logging, Render Loop
The ``main`` function simply creates a new ``Application`` object and calls the ``Start()`` function provided below.

Logging is quite important for a project of this size, using spdlog and a few ``#define`` directives makes logging events of different priorities very simple.

A ``Window`` class is responsible for laying out the GUI of the app: drawing panels, displaying the scene preview etc.

A ``renderer`` is also created here and passed to the window object. The window depends on it because it must retrieve the pixel buffer to draw the rasterized image every frame.

The render loop actively checks for the closing of the GUI window while updating the window and itself.

```cpp
int Application::Start(std::string title, const unsigned int width, const unsigned int height, bool enableVsync)
{
    Logger::Init();
    LOG_INFO("Logging enabled. Welcome to my rasterizer!");

    this->window = std::make_unique<Window>(title, width, height, enableVsync);

    LOG_INFO("Creating OpenGL renderer instance");
    renderer = std::make_shared<OpenGlRenderer>(width, height);
    window->setRenderer(renderer);

    LOG_INFO("Starting main application loop");
    while (!this->shouldClose())
    {
        window->onUpdate();
        this->onUpdate();
    }
    LOG_WARN("Shutting down application...");
    window = nullptr;
    return 0;
```

## Renderer API
One of the more interesting aspects of such a program is its approach to the **Renderer API Abstraction**. The engine should provide abstractions that can be easily adapted with different backends like OpenGL or DirectX. And while I have no intentions of ever supporting anything other than OpenGL, such as DirectX or Vulkan, this is still quite relevant when ray tracing will be implemented.

But even within OpenGL, there are situations where multiple renders are needed. For example rendering material previews.

For now this part is not quite developed. At the moment I cannot justify the need for a ``Renderer`` base class with concrete renderers that inherit from it simply because the abstractions need to be really well thought out (since different render backends behave differently).

The only main renderer implementation is the ``OpenGlRenderer`` class. In it's constructor it compiles all the OpenGL shaders required: lights, grids, meshes etc. and insures all of them compiled correctly. It also initializes a ``SceneBuffer`` object which stores the output of the renderer.

```cpp
OpenGlRenderer::OpenGlRenderer(float width, float height)
{
    LOG_INFO("Compiling shaders...");

    int allCompiled = 1;
    int ok = gridShader.init("../shaders/grid.vert", "../shaders/grid.frag");
    
    if (!ok)
        LOG_CRITICAL("Failed to compile grid shader");
    initGrid();

   // ...

    this->width = width;
    this->height = height;
    sceneBuffer.Init(this->width, this->height);

    glEnable(GL_DEPTH_TEST);
}
```

For each frame, the renderer fetches all objects it needs from the ``SceneManager`` singleton object. It calculates model, projection and view matrices, activates appropriate shaders,sets uniformd, binds appropiate VAOs and calls OpenGL ``draw`` functions.

```cpp
void OpenGlRenderer::Render()
{
    //...

    sceneBuffer.Bind();
    glClearColor(bgColor[0], bgColor[1], bgColor[2], bgColor[3]);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glm::mat4 model = glm::mat4(1.0f);
    glm::mat4 projection = camera->GetProjectionMatrix(width, height);
    glm::mat4 view = camera->GetViewMatrix();

    if (drawGrid)
    {
        gridShader.use();
        gridShader.setMat4("projection", projection);
        gridShader.setMat4("view", view);
        gridShader.setMat4("model", model);
        glBindVertexArray(GRID_VAO);
        glDrawElements(GL_LINES, GRID_LEN, GL_UNSIGNED_INT, NULL);
        glBindVertexArray(0);
    }

    meshShader.use();
    meshShader.setInt("numPointLights", SceneManager::GetInstance()->nrOfPointLights);
    meshShader.setInt("numDirLights", SceneManager::GetInstance()->nrOfDirLights);
    meshShader.setInt("numSpotLights", SceneManager::GetInstance()->nrOfSpotLights);
    for (int i = 0; i < SceneManager::GetInstance()->lights.size(); i++)
    {
        auto light = SceneManager::GetInstance()->lights[i];
        light->setUniforms(meshShader, i);
    }

    meshShader.setMat4("projection", projection);
    meshShader.setMat4("view", view);

    for (int i = 0; i < SceneManager::GetInstance()->models.size(); i++)
    {
        auto model = SceneManager::GetInstance()->models[i];
        auto modelMatrix = model->getTransform();
        meshShader.setMat4("model", modelMatrix);
        model->Draw(meshShader);
    }

    sceneBuffer.Unbind();
}
```
![backpack](backpack.png)
_Early version: no lights or materials, just model import with textures_

## Entities
This is again of the more complicated decisions one has to make about the architecture of such an engine. Each mesh, light, material, primitive, polygon etc. can be considered an *entity* in the world. That means some functionality is shared among them, while a lot of other functionality isn't (for example a Point Light is similar to an Area Light in its position, color, angle, intensity etc., but quite different when it comes to light calculations).

This seems like a perfect recipe for writing yet another inheritance hierarchy, but **no**. That's a bad idea. And while it's not as complex as when building a game engine, different entities can be composed in interesting ways. For example, a light *may* have a material assigned to it, or a 3D mesh can act as a light source. It's very difficult to describe such compositions with inheritance. 

Which is why a better approach is to use **Composition over inheritance**.

At the moment I still use inheritance though: a base Entity class has a name and ID, which is a unique random string and other classes that derive from it.

```cpp
class Entity
{
public:
    Entity(EntityType type);
    Entity(EntityType type, std::string name);
    virtual ~Entity();

    void setName(std::string newName) { this->name = newName; }
    std::string getName() { return this->name; };

    std::vector<std::shared_ptr<Entity>> &getChildren() { return children; }

    EntityType getType() { return type; }

    std::string getUUID() { return UUID; }
private:
    std::string name;
    std::string UUID;

    std::vector<std::shared_ptr<Entity>> children;

    std::string generateUUID()
    {
       //...
    }
};
```

I plan to migrate my entities to EnTT very soon, which is an open source Entity Component System (ECS) built mainly for games.

## Models, materials, lights
I use the Assimp Library to automatically parse different 3D model files and load them into data structures. I then take the data and pass it to OpenGL for rendering.

Physically Based Rendering is one of the primary features I wanted to implement, but at the moment I still only have Phong materials. There was no point to progress with PBR without having other basic features of the engine working.
![sphere](sphere.png)
_Tesselated sphere with a diffuse white Phong material, lit by two point lights_

Here is a snippet of the mesh fragment shader that deals with calculating the color of a pixel based on one directional light:
```glsl
// Calculates the color of pixel when using a directional light.
vec3 Directional(DirLight light, vec3 normal, vec3 viewDir)
{
    vec3 lightDir = normalize(-light.direction);
    // diffuse shading
    float diff = max(dot(normal, lightDir), 0.0);
    // specular shading
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
    // combine results
    vec3 ambient;
    if (material.useDiffuseMap)
    {
        vec4 t = texture(material.diffuseMap, TexCoords);
        if (t.a < 0.1)
            discard;
        ambient = vec3(t);
    }
    else
        ambient = material.ambient;

    vec3 diffuse;
    if (material.useDiffuseMap)
        diffuse = light.color * diff * vec3(texture(material.diffuseMap, TexCoords));
    else
        diffuse = light.color * diff * material.diffuse;

    vec3 specular;
    if (material.useSpecularMap)
        specular = light.color * spec * vec3(texture(material.specularMap, TexCoords));
    else
        specular = light.color * spec * material.specular;

    return (material.ka * ambient + material.kd * diffuse + material.ks * specular);
}
```