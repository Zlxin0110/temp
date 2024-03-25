在GitHub上创建一个新的仓库：
登录GitHub账号。
点击页面右上角的加号图标，选择“New repository”。
输入仓库名称和描述，选择公开或私有，然后点击“Create repository”。


打开终端，进入ROS包的目录。
运行以下命令将ROS包初始化为Git仓库：
git init


将ROS包的文件添加到Git仓库：
运行以下命令将所有文件添加到Git仓库：
git add .
运行以下命令将指定文件夹下的文件添加到Git仓库：
git add my_folder/


提交更改：
运行以下命令提交文件到Git仓库：
git commit -m "comment"
