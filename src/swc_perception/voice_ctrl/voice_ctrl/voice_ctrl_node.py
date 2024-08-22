import rclpy
from rclpy.node import Node
import subprocess

class FileReaderNode(Node):
    def __init__(self):
        super().__init__('file_reader_node')
        self.filename = "/tmp/voice_recognition_text.txt"
        self.timer = self.create_timer(0.1, self.timer_callback)  # 每隔 0.1 秒触发一次回调函数

        self.last_command = None

        # 先删掉历史的命令文件
        try:
            subprocess.run(['sudo', 'rm', '-f', self.filename], check=True)
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Command failed with error: {e}")


    def timer_callback(self):
        # 读取文件的第三行
        third_line = self.read_third_line(self.filename)

        if third_line:
            # 提取 <rawtext> 标签之间的内容
            extracted_text = self.extract_text_between_tags(third_line)
            if extracted_text == self.last_command:
                self.get_logger().info(f"No new command, last command is {self.last_command}")
                return

            self.last_command = extracted_text

            # 检查是否是“前进”
            if extracted_text == "前进":
                self.get_logger().info("Detected '前进' in the text.")
            elif extracted_text == "前进":
                self.get_logger().info("Detected '后退' in the text.")
            elif extracted_text == "左转":
                self.get_logger().info("Detected '左转' in the text.")
            elif extracted_text == "右转":
                self.get_logger().info("Detected '右转' in the text.")
            else:
                self.get_logger().info(f"Text between <rawtext> tags: {extracted_text}")

        else:
            self.get_logger().info("Third line not found or file is empty.")

    def read_third_line(self, filename):
        try:
            with open(filename, 'r') as file:
                lines = file.readlines()
                if len(lines) >= 3:
                    return lines[2].strip()  # 返回第三行内容并去掉多余的空白符
                else:
                    return ""
        except IOError as e:
            self.get_logger().error(f"Error reading file: {e}")
            return ""

    def extract_text_between_tags(self, line):
        start_tag = "<rawtext>"
        end_tag = "</rawtext>"

        start_pos = line.find(start_tag)
        if start_pos == -1:
            return ""  # 如果找不到开始标签，返回空字符串
        start_pos += len(start_tag)  # 移动到标签之后的位置

        end_pos = line.find(end_tag, start_pos)
        if end_pos == -1:
            return ""  # 如果找不到结束标签，返回空字符串

        return line[start_pos:end_pos]  # 返回两个标签之间的内容
    

def main(args=None):
    rclpy.init(args=args)
    file_reader_node = FileReaderNode()
    rclpy.spin(file_reader_node)
    file_reader_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
