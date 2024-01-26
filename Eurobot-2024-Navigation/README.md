# Eurobot-2024-Navigation

## First Time Setup

### Docker

If you're using docker, you can get [dockerfile (created by YuZhong Cheng)](https://drive.google.com/drive/folders/1KZFPPq6YLLryNOcabEBwlLQIlN6rCTO5?usp=sharing).

### Install & Build

```bash
# Create src folder (Under your workspace)
mkdir src

git clone git@github.com:DIT-ROBOTICS/Eurobot-2024-Navigation.git src/Eurobot-2024-Navigation

# Create your branch (Under Eurobot-2024-Navigation folder)
git branch <your-name-for-branch>
git checkout <your-name-for-branch>

# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro noetic -y

# Compile the project (Under workspace)
catkin_make
```

### Modify & Push

```bash
# Under Eurobot-2024-Navigation folder
git pull # get updates from github

# MAKE SOME COMMITS ON YOUR BRANCH
 
git remote add <remote-name> <https/ssh method>
git push <remote-name> <your-name-for-branch>
```
