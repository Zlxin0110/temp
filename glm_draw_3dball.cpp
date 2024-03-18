#include <iostream>
#include <thread>
#include <GL/glew.h>
//#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <chrono>
#include <vector>
#include <png++/png.hpp>
#include "shader.h"
#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

const GLuint WIDTH = 800, HEIGHT = 600;
const float PI = 3.14159265359f;

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

void processInput(GLFWwindow* window) {
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
}

const char* vertexShaderSource = R"(
    #version 330 core
    layout (location = 0) in vec3 aPos;

    uniform mat4 model;
    uniform mat4 view;
    uniform mat4 projection;
    out vec3 FragPos;
    void main() {
        gl_Position = projection * view * model * vec4(aPos, 1.0);
        FragPos = aPos;
    }
)";

const char* fragmentShaderSource = R"(
    #version 330 core
    out vec4 FragColor;

    void main() {
        FragColor = vec4(1.0, 0.5, 0.2, 1.0);
    }
)";

int main() {
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(WIDTH, HEIGHT, "3D Sphere", NULL, NULL);
    if (window == NULL) {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK) {
        std::cout << "Failed to initialize GLEW" << std::endl;
        return -1;
    }

    glViewport(0, 0, WIDTH, HEIGHT);
// ++++++++++++++++++++++++++++++++++++++
const float axisVertices[] = {
    // X轴
    0.0f, 0.0f, 0.0f,  1.0f, 0.0f, 0.0f,
    1.0f, 0.0f, 0.0f,  1.0f, 0.0f, 0.0f,

    // Y轴
    0.0f, 0.0f, 0.0f,  0.0f, 1.0f, 0.0f,
    0.0f, 1.0f, 0.0f,  0.0f, 1.0f, 0.0f,

    // Z轴
    0.0f, 0.0f, 0.0f,  0.0f, 0.0f, 1.0f,
    0.0f, 0.0f, 1.0f,  0.0f, 0.0f, 1.0f
};

const char* axisVertexShaderSource = R"(
    #version 330 core
    layout (location = 0) in vec3 aPos;
    layout (location = 1) in vec3 aColor;

    out vec3 color;

    uniform mat4 model;
    uniform mat4 view;
    uniform mat4 projection;

    void main() {
        gl_Position = projection * view * model * vec4(aPos, 1.0);
        color = aColor;
    }
)";

const char* axisFragmentShaderSource = R"(
    #version 330 core
    in vec3 color;
    out vec4 FragColor;

    void main() {
        FragColor = vec4(color, 1.0);
    }
)";

// 创建和绑定VAO和VBO
GLuint axisVAO, axisVBO;
glGenVertexArrays(1, &axisVAO);
glGenBuffers(1, &axisVBO);
glBindVertexArray(axisVAO);
glBindBuffer(GL_ARRAY_BUFFER, axisVBO);
glBufferData(GL_ARRAY_BUFFER, sizeof(axisVertices), axisVertices, GL_STATIC_DRAW);
glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
glEnableVertexAttribArray(0);
glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
glEnableVertexAttribArray(1);

// 创建和编译坐标轴着色器
GLuint axisVertexShader = glCreateShader(GL_VERTEX_SHADER);
glShaderSource(axisVertexShader, 1, &axisVertexShaderSource, NULL);
glCompileShader(axisVertexShader);

GLuint axisFragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
glShaderSource(axisFragmentShader, 1, &axisFragmentShaderSource, NULL);
glCompileShader(axisFragmentShader);

GLuint axisShaderProgram = glCreateProgram();
glAttachShader(axisShaderProgram, axisVertexShader);
glAttachShader(axisShaderProgram, axisFragmentShader);
glLinkProgram(axisShaderProgram);

glDeleteShader(axisVertexShader);
glDeleteShader(axisFragmentShader);
// ++++++++++++++++++++++++++++++++++++++
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
    glCompileShader(vertexShader);

    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
    glCompileShader(fragmentShader);

    GLuint shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    // Create sphere vertices
    const int sectorCount = 100;
    const int stackCount = 100;
    std::vector<float> vertices;
    for (int i = 0; i <= stackCount; ++i) {
        float stackAngle = PI / 2 - i * PI / stackCount;
        float xy = 2.0f * cosf(stackAngle);
        float z = 2.0f * sinf(stackAngle);
        for (int j = 0; j <= sectorCount; ++j) {
            float sectorAngle = j * 2 * PI / sectorCount;
            float x = xy * cosf(sectorAngle);
            float y = xy * sinf(sectorAngle);
            vertices.push_back(x);
            vertices.push_back(y);
            vertices.push_back(z);
        }
    }

    GLuint VBO, VAO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    while (!glfwWindowShouldClose(window)) {
        processInput(window);

        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
// ++++++++++++++++++++++++++++++++++++++
    // 绘制坐标轴
    glUseProgram(axisShaderProgram);
    glm::mat4 axisModel = glm::mat4(1.0f);
    glm::mat4 axisView = glm::lookAt(glm::vec3(5.0f, 5.0f, 10.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    glm::mat4 axisProjection = glm::perspective(glm::radians(45.0f), (float)WIDTH / (float)HEIGHT, 0.1f, 100.0f);
    glUniformMatrix4fv(glGetUniformLocation(axisShaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(axisModel));
    glUniformMatrix4fv(glGetUniformLocation(axisShaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(axisView));
    glUniformMatrix4fv(glGetUniformLocation(axisShaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(axisProjection));

    glBindVertexArray(axisVAO);
    glDrawArrays(GL_LINES, 0, 6);
    glBindVertexArray(0);
// ++++++++++++++++++++++++++++++++++++++
        glUseProgram(shaderProgram);
        glm::mat4 model = glm::mat4(1.0f);
        glm::mat4 view = glm::lookAt(glm::vec3(5.0f, 5.0f, 10.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        glm::mat4 projection = glm::perspective(glm::radians(45.0f), (float)WIDTH / (float)HEIGHT, 0.1f, 100.0f);
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));

        glBindVertexArray(VAO);
        glDrawArrays(GL_POINTS, 0, vertices.size() / 3);
        glBindVertexArray(0);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteProgram(shaderProgram);

    glfwTerminate();
    return 0;
}
https://glm.g-truc.net/0.9.8/index.html


#include <iostream>
#include <thread>
#include <GL/glew.h>
//#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <chrono>
#include <vector>
#include <png++/png.hpp>
#include "shader.h"
#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
// ======================================================================
const GLuint WIDTH = 800, HEIGHT = 600;
const float PI = 3.14159265359f;
const int sectorCount = 100;
const int stackCount = 100;

// ======================================================================
// 生成3D点云数据
void saveRenderbufferToPNG(const std::string& filename, int width, int height) {
    std::vector<unsigned char> pixels(width * height * 4); // RGBA format
    glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());

    png::image<png::rgba_pixel> image(width, height);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int index = (y * width + x) * 4;
            image[y][x] = png::rgba_pixel(pixels[index], pixels[index + 1], pixels[index + 2], pixels[index + 3]);
        }
    }
    image.write(filename);
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

void processInput(GLFWwindow* window) {
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
}

const char* vertexShaderSource = R"(
    #version 330 core
    layout (location = 0) in vec3 aPos;

    uniform mat4 model;
    uniform mat4 view;
    uniform mat4 projection;
    out vec3 FragPos;
    void main() {
        gl_Position = projection * view * model * vec4(aPos, 1.0);
        FragPos = aPos;
    }
)";

const char* fragmentShaderSource = R"(
    #version 330 core
    out vec4 FragColor;

    void main() {
        FragColor = vec4(1.0, 0.5, 0.2, 1.0);
    }
)";

const char* axisVertexShaderSource = R"(
    #version 330 core
    layout (location = 0) in vec3 aPos;
    layout (location = 1) in vec3 aColor;

    out vec3 color;

    uniform mat4 model;
    uniform mat4 view;
    uniform mat4 projection;

    void main() {
        gl_Position = projection * view * model * vec4(aPos, 1.0);
        color = aColor;
    }
)";

const char* axisFragmentShaderSource = R"(
    #version 330 core
    in vec3 color;
    out vec4 FragColor;

    void main() {
        FragColor = vec4(color, 1.0);
    }
)";

const char* screenVertexShaderSource = R"(
    #version 330 core
    layout (location = 0) in vec2 aPos;
    layout (location = 1) in vec2 aTexCoord;

    out vec2 TexCoord;

    void main() {
        gl_Position = vec4(aPos, 0.0, 1.0);
        TexCoord = aTexCoord;
    }
)";

const char* screenFragmentShaderSource = R"(
    #version 330 core
    out vec4 FragColor;

    in vec2 TexCoord;

    uniform sampler2D texture1;

    void main() {
        FragColor = texture(texture1, TexCoord);
    }
)";

GLFWwindow* createWindow()
{
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(WIDTH, HEIGHT, "3D Sphere", NULL, NULL);
    if (window == NULL) {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return nullptr;
    }

    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK) {
        std::cout << "Failed to initialize GLEW" << std::endl;
        return nullptr;
    }

    return window;
}

const float axisVertices[] = {
    // X轴
    0.0f, 0.0f, 0.0f,  1.0f, 0.0f, 0.0f,
    1.0f, 0.0f, 0.0f,  1.0f, 0.0f, 0.0f,

    // Y轴
    0.0f, 0.0f, 0.0f,  0.0f, 1.0f, 0.0f,
    0.0f, 1.0f, 0.0f,  0.0f, 1.0f, 0.0f,

    // Z轴
    0.0f, 0.0f, 0.0f,  0.0f, 0.0f, 1.0f,
    0.0f, 0.0f, 1.0f,  0.0f, 0.0f, 1.0f
};

// 定义矩形的顶点数据和纹理坐标
float textureVertices[] = {
    // 位置          // 纹理坐标
    -1.0f, -1.0f, 0.0f, 0.0f,
     1.0f, -1.0f, 1.0f, 0.0f,
     1.0f,  1.0f, 1.0f, 1.0f,

    -1.0f, -1.0f, 0.0f, 0.0f,
     1.0f,  1.0f, 1.0f, 1.0f,
    -1.0f,  1.0f, 0.0f, 1.0f
};

GLuint createShaderProgram(const char* vertexs,const char* fragments)
{
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexs, NULL);
    glCompileShader(vertexShader);

    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragments, NULL);
    glCompileShader(fragmentShader);

    GLuint shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    return shaderProgram;
}

std::vector<float> getBallData(const int sectorCount, const int stackCount)
{
    std::vector<float> vertices;
    for (int i = 0; i <= stackCount; ++i) {
        float stackAngle = PI / 2 - i * PI / stackCount;
        float xy = 2.0f * cosf(stackAngle);
        float z = 2.0f * sinf(stackAngle);
        for (int j = 0; j <= sectorCount; ++j) {
            float sectorAngle = j * 2 * PI / sectorCount;
            float x = xy * cosf(sectorAngle);
            float y = xy * sinf(sectorAngle);
            vertices.push_back(x);
            vertices.push_back(y);
            vertices.push_back(z);
        }
    }

    return vertices;
}

int main()
{
    GLFWwindow* window = createWindow();
    if (window == NULL) {
        return -1;
    }

    glViewport(0, 0, WIDTH, HEIGHT);
    // ++++++++++++++++++++++++++++++++++++++
    // 创建和编译坐标轴着色器
    GLuint textureShaderProgram = createShaderProgram(screenVertexShaderSource, screenFragmentShaderSource);
    // 创建和编译坐标轴着色器
    GLuint axisShaderProgram = createShaderProgram(axisVertexShaderSource, axisFragmentShaderSource);
    // 创建和编译ball着色器
    GLuint shaderProgram = createShaderProgram(vertexShaderSource, fragmentShaderSource);
    // =================================================================================
    // Create sphere vertices
    std::vector<float> vertices = getBallData(sectorCount,stackCount);
// =================================================================================
GLuint framebuffer;
glGenFramebuffers(1, &framebuffer);
glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);

GLuint texture;
glGenTextures(1, &texture);
glBindTexture(GL_TEXTURE_2D, texture);
glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, WIDTH, HEIGHT, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture, 0);

if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
    std::cout << "Error: Framebuffer is not complete!" << std::endl;
}

glBindFramebuffer(GL_FRAMEBUFFER, 0);


// 创建并绑定 VAO
GLuint textureVAO, textureVBO;
glGenVertexArrays(1, &textureVAO);
glGenBuffers(1, &textureVBO);
glBindVertexArray(textureVAO);
glBindBuffer(GL_ARRAY_BUFFER, textureVBO);
glBufferData(GL_ARRAY_BUFFER, sizeof(textureVertices), textureVertices, GL_STATIC_DRAW);
glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
glEnableVertexAttribArray(0);
glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));
glEnableVertexAttribArray(1);
// glBindBuffer(GL_ARRAY_BUFFER, 0);
// glBindVertexArray(0);

// =================================================================================
    // 创建和绑定VAO和VBO
    GLuint axisVAO, axisVBO;
    glGenVertexArrays(1, &axisVAO);
    glGenBuffers(1, &axisVBO);
    glBindVertexArray(axisVAO);
    glBindBuffer(GL_ARRAY_BUFFER, axisVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(axisVertices), axisVertices, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
// =================================================================================
    GLuint VBO, VAO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
// =================================================================================
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    while (!glfwWindowShouldClose(window)) {
        processInput(window);

        // glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        // glClear(GL_COLOR_BUFFER_BIT);
// ====================================================================
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
// ====================================================================

// ++++++++++++++++++++++++++++++++++++++
    // 绘制坐标轴
    glUseProgram(axisShaderProgram);
    glm::mat4 axisModel = glm::mat4(1.0f);
    glm::mat4 axisView = glm::lookAt(glm::vec3(5.0f, 5.0f, 10.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    glm::mat4 axisProjection = glm::perspective(glm::radians(45.0f), (float)WIDTH / (float)HEIGHT, 0.1f, 100.0f);
    glUniformMatrix4fv(glGetUniformLocation(axisShaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(axisModel));
    glUniformMatrix4fv(glGetUniformLocation(axisShaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(axisView));
    glUniformMatrix4fv(glGetUniformLocation(axisShaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(axisProjection));

    // glBindVertexArray(axisVAO);
    // glDrawArrays(GL_LINES, 0, 6);
    // glBindVertexArray(0);
glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
glBindVertexArray(axisVAO);
glDrawArrays(GL_LINES, 0, 6);
glBindVertexArray(0);
// ++++++++++++++++++++++++++++++++++++++
        glUseProgram(shaderProgram);
        glm::mat4 model = glm::mat4(1.0f);
        glm::mat4 view = glm::lookAt(glm::vec3(5.0f, 5.0f, 10.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        glm::mat4 projection = glm::perspective(glm::radians(45.0f), (float)WIDTH / (float)HEIGHT, 0.1f, 100.0f);
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));

        // glBindVertexArray(VAO);
        // glDrawArrays(GL_POINTS, 0, vertices.size() / 3);
        // glBindVertexArray(0);
glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
glBindVertexArray(VAO);
glDrawArrays(GL_POINTS, 0, vertices.size() / 3);
glBindVertexArray(0);
// 生成3D点云数据

saveRenderbufferToPNG("filename.png", 800, 600);

// ====================================================================
    // 使用纹理渲染到屏幕上
    glUseProgram(textureShaderProgram);
    glBindVertexArray(textureVAO);
    glBindTexture(GL_TEXTURE_2D, texture);
    glDrawArrays(GL_TRIANGLES, 0, 6);
// ====================================================================
        glfwSwapBuffers(window);
        glfwPollEvents();
        break;
    }

    // destory asix resource
    glDeleteVertexArrays(1, &axisVAO);
    glDeleteBuffers(1, &axisVBO);
    glDeleteProgram(axisShaderProgram);
    // destory ball resource
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteProgram(shaderProgram);
    // destory texture resource
    glDeleteVertexArrays(1, &textureVAO);
    glDeleteBuffers(1, &textureVBO);
    glDeleteProgram(textureShaderProgram);
    glDeleteTextures(1, &texture);
    glDeleteFramebuffers(1, &framebuffer);

    glfwTerminate();
    return 0;
}
