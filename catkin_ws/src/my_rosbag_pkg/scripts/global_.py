import os
from datetime import datetime

#relative path to package
package_path = os.path.join(os.path.dirname(__file__), "..") 

# Get the current date and time
current_datetime = datetime.now()

# Format the current date and time as "Year-Month-Date-Hour-Minute-Second"
directory_name = current_datetime.strftime("%Y_%m_%d_%H_%M_%S")

#relative path for each new rosbag record
new_directory = os.path.join(package_path,"bagfiles", directory_name)



