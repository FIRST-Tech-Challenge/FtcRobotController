## This file includes important information on file management and workflow.

Please make sure that you read this document thoroughly before adding new files or pushing changes onto the master. 
Any questions about file hierarchy and workflow can be asked in person at a meeting, or in the coding channel on Discord.

## File Management:

1.  Ensure that everything goes into the pertinent folders. If you have questions about this, ask Atharve(Art) or William.
2.  Every new function must be tested individually before insertion into experimental OpMode.
3.  Every new function must have an accompanying markdown(.MD) file detailing the intended use, along with comments about
    known issues.
4.  Once a function has been tested and troubleshooted thoroughly, it can be moved into the tested folder.
    A detailed comment must exist at the beginning of each file talking about purpose.
5.  Once individually tested, multiple functions can be combined in a file in the experimental folder.
6.  Multiple functions should be compiled into the same OpMode in the experimental folder in order to test together.
7.  A detailed (.MD) file must accompany each experimental OpMode including purpose, successes and error reports.
8.  Once an experimental OpMode has been thoroughly tested, get approval from Art before transferring the OpMode and
    support files into the competition folder.
9.  All OpModes and supporting modules will be copied manually into the competition folder after approval from Art and William.

## Workflow:

1.  Ensure that you create a branch for your devices and always push edits onto the branch instead of the master. 
2.  When you want to push changes to the master, confirm it with Art and make sure that your change list does NOT
    include any gradle or (.XML) changes. The only files your changelist should update will be in the teamcode folder.