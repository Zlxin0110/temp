
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


============================
std::vector<unsigned char> compressData(const std::vector<unsigned char>& data) {
    // 压缩级别
    int level = Z_DEFAULT_COMPRESSION;

    // 分配足够的空间来存储压缩后的数据
    uLongf compressedSize = compressBound(data.size());
    std::vector<unsigned char> compressedData(compressedSize);

    // 压缩数据
    if (compress(compressedData.data(), &compressedSize, data.data(), data.size()) != Z_OK) {
        std::cout << "Failed to compress data." << std::endl;
        return std::vector<unsigned char>();
    }

    // 调整压缩后的数据大小
    compressedData.resize(compressedSize);

    return compressedData;
}


    
    // 初始化ROS节点
    ros::init(argc, argv, "texture_publisher");
    ros::NodeHandle nh;


    std::vector<unsigned char> pixels(WIDTH * HEIGHT * 4); // RGBA format
    glReadPixels(0, 0, WIDTH, HEIGHT, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());

    std::cout << "pixels.size() : " <<  pixels.size() << std::endl;
    
    std::vector<unsigned char> compressedPixels = compressData(pixels);
    std::cout << "compressedPixels.size() : " <<  compressedPixels.size() << std::endl;
    // 将压缩后的像素数据发布到ROS话题
    ros::Publisher pub = nh.advertise<std_msgs::UInt8MultiArray>("compressed_texture_pixels", 1);

    std_msgs::UInt8MultiArray msg;
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].size = compressedPixels.size();
    msg.layout.dim[0].stride = 1;
    msg.layout.dim[0].label = "pixels";
    msg.data = compressedPixels;
    
============================
#include <zlib.h>
std::vector<unsigned char> decompressData(const std::vector<unsigned char>& compressedData) {
    // 分配足够的空间来存储解压缩后的数据
    uLongf uncompressedSize = /* 您需要知道解压缩后的数据大小 */;
    std::vector<unsigned char> uncompressedData(uncompressedSize);

    // 解压缩数据
    if (uncompress(uncompressedData.data(), &uncompressedSize, compressedData.data(), compressedData.size()) != Z_OK) {
        ROS_ERROR("Failed to decompress data.");
        return std::vector<unsigned char>();
    }

    // 调整解压缩后的数据大小
    uncompressedData.resize(uncompressedSize);

    return uncompressedData;
}


// ROS コールバック関数、受信したポイントクラウドデータを処理します
void lidarCallback(const sensor_msgs::UInt8MultiArray::ConstPtr& msg) {
    size_t dataSize = msg->data.size();
    std::cout << "dataSize : " << dataSize << std::endl;

    // 解压缩消息数据
    std::vector<unsigned char> uncompressedData = decompressData(msg->data);
    std::cout << "uncompressedData : " << uncompressedData << std::endl;
}
// メイン関数
int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_visualization_node");
    ros::NodeHandle nh;

    // 点群トピックを購読し、コールバック関数を設定します
    ros::Subscriber lidar_sub = nh.subscribe("/compressed_texture_pixels", 1, lidarCallback);

    // ROSメインループ
    ros::spin();

    // OpenGLスレッドの終了を待ちます
    gl_thread.join();

    return 0;
}
