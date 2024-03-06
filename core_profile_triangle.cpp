
// 顶点着色器代码
const char* vertexShaderSource = R"(
    #version 330 core
    layout (location = 0) in vec3 aPos;
    layout (location = 1) in vec3 aColor;

    out vec3 ourColor;

    void main()
    {
        gl_Position = vec4(aPos, 1.0);
        ourColor = aColor;
    }
)";

// 片段着色器代码
const char* fragmentShaderSource = R"(
    #version 330 core
    out vec4 FragColor;

    in vec3 ourColor;

    void main()
    {
        FragColor = vec4(ourColor, 1.0f);
    }
)";

int main()
{
    // 初始化GLFW
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // 创建窗口对象
    GLFWwindow* window = glfwCreateWindow(800, 600, "OpenGL", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    // 初始化GLEW
    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK)
    {
        std::cout << "Failed to initialize GLEW" << std::endl;
        return -1;
    }

    // 编译顶点着色器
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
    glCompileShader(vertexShader);

    // 编译片段着色器
    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
    glCompileShader(fragmentShader);

    // 创建着色器程序
    GLuint shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);

    // 删除着色器
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    // 顶点数据
    float vertices[] = {
        // 位置              // 颜色
         0.0f,  0.5f, 0.0f,  1.0f, 0.0f, 0.0f, // 上顶点红色
        -0.5f, -0.5f, 0.0f,  0.0f, 1.0f, 0.0f, // 左下角绿色
         0.5f, -0.5f, 0.0f,  0.0f, 0.0f, 1.0f  // 右下角蓝色
    };

    // 创建顶点数组对象和顶点缓冲对象
    GLuint VAO, VBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    // 绑定顶点数组对象和顶点缓冲对象
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    // 设置顶点属性指针
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    // 创建帧缓冲对象
    unsigned int framebuffer;
    glGenFramebuffers(1, &framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);

    // 创建纹理附件
    unsigned int texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 800, 600, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture, 0);

    // 检查帧缓冲完整性
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        std::cout << "Framebuffer is not complete!" << std::endl;
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

        // 渲染到纹理
        glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
        glClearColor(1.0f, 0.0f, 0.0f, 1.0f); // 红色背景
        glClear(GL_COLOR_BUFFER_BIT);
        glUseProgram(shaderProgram);
        glBindVertexArray(VAO);
        glDrawArrays(GL_TRIANGLES, 0, 3);
saveRenderbufferToPNG("test1.png", WIDTH, HEIGHT);

// 渲染到屏幕
glBindFramebuffer(GL_FRAMEBUFFER, 0);
//glClearColor(1.0f, 1.0f, 1.0f, 1.0f); // 白色背景
glClear(GL_COLOR_BUFFER_BIT);
glUseProgram(shaderProgram); // 使用纹理的着色器程序
glBindVertexArray(VAO);
glBindTexture(GL_TEXTURE_2D, texture); // 将纹理绑定到纹理单元
glDrawArrays(GL_TRIANGLES, 0, 3);
saveRenderbufferToPNG("test2.png", WIDTH, HEIGHT);

        // 交换缓冲区和检查事件
        glfwSwapBuffers(window);
    // 渲染循环
    while (!glfwWindowShouldClose(window))
    {
        // 输入事件处理
        glfwPollEvents();
    }

    // 清理资源
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteProgram(shaderProgram);

    // 终止GLFW
    glfwTerminate();
    return 0;
}
