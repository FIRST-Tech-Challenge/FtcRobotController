// go to code base directory
cd <base directory> (eg. C:\Users\komal\Documents\GitHub\FTCTeamRepo\FtcRobotController)

// check git status
git status

// checkout master
git checkout master

// check git status to make sure its good
git status

// pull and rebase
git pull --rebase

// cretae your branch
git checkout -b <branch_name> (eg. git checkout -b Autonomous_Blue)

// now make code changes

// when ready come back and commit your changes after testing
git commit -a

// check git status
git status

//push your changes to main or upstream
git push --set-upstream origin <branch_name> (eg. git push --set-upstream origin Autonomous_Blue)

// finally create a pull request or PR
// go to brower, create a merge request for https://github.com/botsandbytes/intothedeep/ repo only by changing left side and the rebase + merge and then delte branch and start again from start
