#include <iostream>
#include <thread>
#include <GL/glew.h>
//#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <chrono>
#include <vector>
#include <png++/png.hpp>
#include "shader.h"



const char* vertexShaderSource = R"(
    #version 330 core
    layout (location = 0) in vec3 aPos;
    layout (location = 1) in vec3 aColor;

    out vec3 Color;
    out vec2 TexCoord;

    void main()
    {
        gl_Position = vec4(aPos, 1.0);
        Color = aColor;
        TexCoord = vec2((aPos.x + 1.0) / 2.0, 1.0 - (aPos.y + 1.0) / 2.0);
    }
)";


const char* fragmentShaderSource = R"(
    #version 330 core
    in vec3 Color;
    out vec4 FragColor;

    void main()
    {
        FragColor = vec4(Color, 1.0);
    }
)";

// 生成点云数据
std::vector<float> generatePointCloud(int width, int height)
{
    std::vector<float> points;
    for (int i = 0; i < width; ++i)
    {
        for (int j = 0; j < height; ++j)
        {
            // 生成随机颜色
            float r = static_cast<float>(rand()) / RAND_MAX;
            float g = static_cast<float>(rand()) / RAND_MAX;
            float b = static_cast<float>(rand()) / RAND_MAX;
            points.push_back((float)i / width * 2 - 1);   // x 坐标，将[0,1]范围映射到[-1,1]
            points.push_back((float)j / height * 2 - 1);  // y 坐标，将[0,1]范围映射到[-1,1]
            points.push_back(0.0f);               // z 坐标
            points.push_back(r);                  // r 分量
            points.push_back(g);                  // g 分量
            points.push_back(b);                  // b 分量
        }
    }
    return points;
}

int main()
{
    // 初始化GLFW
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // 创建窗口
    GLFWwindow* window = glfwCreateWindow(800, 600, "OpenGL", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    // 初始化GLEW
    if (glewInit() != GLEW_OK)
    {
        std::cout << "Failed to initialize GLEW" << std::endl;
        return -1;
    }

    // 创建顶点着色器
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
    glCompileShader(vertexShader);

    // 创建片段着色器
    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
    glCompileShader(fragmentShader);

    // 创建着色器程序
    GLuint shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);

    // 生成点云数据
    int textureWidth = 80;
    int textureHeight = 60;
    std::vector<float> points = generatePointCloud(textureWidth, textureHeight);

    // 创建并绑定纹理
    GLuint textureID;
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, textureWidth, textureHeight, 0, GL_RGB, GL_FLOAT, points.data());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    // 创建顶点数组对象（VAO）和顶点缓冲对象（VBO）
    GLuint VAO, VBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    // 绑定VAO和VBO
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * points.size(), points.data(), GL_STATIC_DRAW);

    // 设置顶点属性指针
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    // 主循环
    while (!glfwWindowShouldClose(window))
    {
        // 渲染
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        // 使用着色器程序
        glUseProgram(shaderProgram);

        // 绘制点云
        glBindTexture(GL_TEXTURE_2D, textureID);
        glBindVertexArray(VAO);
        glDrawArrays(GL_POINTS, 0, textureWidth * textureHeight);

        // 交换缓冲区和检查事件
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // 清理资源
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteProgram(shaderProgram);
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    glDeleteTextures(1, &textureID);

    // 终止GLFW
    glfwTerminate();
    return 0;
}
