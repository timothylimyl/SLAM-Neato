# MCHA4000 2020 group project template

## Getting started

Each group member should `clone` this repository to a local directory using your favourite git GUI (e.g., SourceTree) or at the command line

    git clone https://bitbucket.org/uonmechatronics/MCHA4000_2020_G#.git
    cd MCHA4000_2020_G#

where `#` is your group number.
Cloning a git repository automatically configures it to use the repository it was cloned from as the remote called `origin`.

Refer to the freely available [Pro Git eBook](https://git-scm.com/book) to familiarise yourself with using `git push` and `git pull` to work with the remote repo and collaborate with your other group members.

## Notes

Review the hidden `.gitignore` file(s) so that you know which file and folder patterns will not be committed. These typically include temporary files, intermediate build files and binary output files.
If you need to change which files are ignored, see the [documentation here](https://git-scm.com/docs/gitignore).

**Write messages as if you are giving orders to the codebase to change its behaviour.**
Remember that a commit is a set of instructions for how to go from a previous state to a new state.
Each commit is an atomic transaction on the codebase that should represent a logically separate changeset that is labelled with a short message (typically less than 50 characters).
Commit messages should be written in the *present-tense imperative*.
For example, write "Fix bug", not "Fixed bug", "Fixes bug", "I fixed the bug" or "This patch fixes the bug" etc.
This writing style makes it easier to `revert`, `rebase` or `cherry-pick` commits later if needed and ensures the messages still make sense when they are used outside their original context.

**Organise your files logically.**
For example, you might consider the following folder structure:

* `MCHA4000-2020-G#`
    * `data/` (the contents are already `.gitignore`d)
    * `doc/` (any documentation generated)
    * `src/` (native source code)
    * `matlab/` (Matlab source and Simulink models)
    * `thirdparty/` (pristine copies of third party dependencies)
    * `README.md` (this README file)

**Don't commit large files to the repository.**
If you are using `git` correctly, your repository size should be significantly [less than 1GB](https://confluence.atlassian.com/bitbucket/what-kind-of-limits-do-you-have-on-repository-file-size-273877699.html).
For comparison, the size of the entire Linux kernel git repository [is only 1.5GB](https://github.blog/2018-03-05-measuring-the-many-sizes-of-a-git-repository/), which includes 25 years of commit history made by thousands of developers.
Presentation media (videos etc.) should be submitted with the presentation slides using Blackboard.
If your project solution relies on a large input dataset (such as with machine vision or LiDAR data), keep it in the `/data` folder, which is already `.gitignore`d.

## Submission instructions

1. Edit/update/replace this `README.md` with suitable instructions to run your solution.
A reference to the basic Markdown syntax can be found [here](https://www.markdownguide.org/basic-syntax/).

1. If your project solution relies on a large input dataset, submit this data separately using the project submission link on Blackboard, and include instructions in `README.md` to indicate that you solution relies on the data contained in this folder.

1. Merge (or rebase) the work your wish to be assessed onto the `master` branch if you haven't already.

1. Push the local `master` branch to the remote `origin/master` branch.

The commit that `origin/master` points to at the due date will considered as the submitted project code.

