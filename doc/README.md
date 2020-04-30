# How to use git to get the latest MG-APP?

## Install Git  

- If you use Debian or Ubuntu Linux, you can install `git` directly through a `sudo apt-get install git`, which is very simple.   

- If you use Windows, you can download the installation program directly from [Git's official website](https://git-scm.com/downloads) and then install it according to the `default options`.  After the installation is complete, click the right mouse button in the folder to find `Git Bash` and open the terminal.  

- After the installation is completed, you still need the last step of setting. Enter in the terminal:   
  `git config --global user.name "Your Name"`  
  `git config --global user.email "email@example.com"`  
## Get and Update MG-APP

- Open the terminal and switch to the directory, then use `git clone` to get the MG-APP code:    
`git clone https://github.com/XiaoGongWei/MG_APP`  
The first git clone takes a long time. (It is best **not to modify** the code in the MG_APP directory, you can **copy to other folder** to modify their code. The MG_APP directory is **only used to get the latest code**.)  
- If you want to update the code, switch to the MG-APP directory. Then use the terminal to execute the `git pull` command to update the code.  
- You can use `git log` to view the current version after updating the MG-APP.  

## Git starter tutorial:

1. CSDN: https://blog.csdn.net/xiaoxiao133/article/details/90486888  
2. The history of the most simple Git tutorial: https://www.liaoxuefeng.com/wiki/896043488029600  
3. Graphics, game learning git: https://learngitbranching.js.org/  
4. Novice tutorial: https://www.runoob.com/git/git-tutorial.html  