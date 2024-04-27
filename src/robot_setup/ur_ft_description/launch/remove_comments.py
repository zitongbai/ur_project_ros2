import re
import xacro
import sys
import argparse
def remove_comments(text):
    pattern = r'<!--(.*?)-->'
    return re.sub(pattern, '', text, flags=re.DOTALL)

if __name__ == '__main__':
    # 接收来自命令行的参数，参数格式与xacro命令行工具相同
    # 例如：python remove_comments.py ur_ft_description.urdf.xacro xx:=yy ...
    # 创建 ArgumentParser 对象
    parser = argparse.ArgumentParser(description='Process some command line arguments.')

    # 添加文件名参数
    parser.add_argument('filename', type=str, help='The filename to process.')

    # 添加可变参数
    parser.add_argument('pairs', nargs='*', type=str, help='The key-value pairs in the format xx:=yy.')

    # 解析命令行参数
    args = parser.parse_args()
    
    # 调用 xacro.process_file() 函数
    urdf = xacro.process_file(args.filename, mappings=dict([pair.split(':=') for pair in args.pairs]))
    urdf_string = urdf.toxml()
    urdf_string = remove_comments(urdf_string)
    print(urdf_string)
    