import rclpy
from rclpy.node import Node

class FileReaderNode(Node):
    def __init__(self):
        super().__init__('file_reader_node')
        self.filename = '/home/jetson/Desktop/SmartWCHR/src/swc_perception/voice_ctrl/resource/raw_text.txt'
        self.timer = self.create_timer(1.0, self.timer_callback)  # 每隔 1 秒触发一次回调函数

    def timer_callback(self):
        # 读取文件的第三行
        third_line = self.read_third_line(self.filename)

        if third_line:
            # 提取 <rawtext> 标签之间的内容
            extracted_text = self.extract_text_between_tags(third_line)

            # 检查是否是“前进”
            if extracted_text == "前进":
                self.get_logger().info("Detected '前进' in the text.")
            else:
                self.get_logger().info(f"Text between <rawtext> tags: {extracted_text}")

            # 清空文件内容
            self.clear_file(self.filename)
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

    def clear_file(self, filename):
        try:
            open(filename, 'w').close()  # 使用 'w' 模式清空文件内容
        except IOError as e:
            self.get_logger().error(f"Error clearing file: {e}")

def main(args=None):
    rclpy.init(args=args)
    file_reader_node = FileReaderNode()
    rclpy.spin(file_reader_node)
    file_reader_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
