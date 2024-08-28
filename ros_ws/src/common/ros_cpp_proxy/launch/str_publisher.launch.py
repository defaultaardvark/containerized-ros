

def generate_launch_description():
    context = LaunchContext()
    

if __name__ == '__main__':
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()