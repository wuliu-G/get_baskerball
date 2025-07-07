# 不要手动执行这个文件，让 catkin 来调用它

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# 从 package.xml 中获取信息并生成 setup() 的参数
setup_args = generate_distutils_setup(
    # 'packages' 列表告诉 Python 你要创建一个叫 'my_pkg1' 的包
    packages=['my_pkg1'],
    # 'package_dir' 是最关键的部分，它告诉 Python:
    # 这个名为 'my_pkg1' 的包，它的源代码在 'scripts' 文件夹里
    package_dir={'': 'scripts'}
)

setup(**setup_args)
