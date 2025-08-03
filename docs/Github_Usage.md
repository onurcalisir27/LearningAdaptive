# Github Usage

## Always start by checking out the latest version, especially if working across different machines
git pull origin <branch_name>               (pull all the changes from the repository)

## After making sure you have all the updates, you can now check what changes you have added
git status                                  (shows what changed but not staged)
git branch                                  (shows the current branch)
git add changed/changed_file                (staged but not added to git)
git add . (all)

## If you want to add your edits to the version control
git commit changed/changed_file -m          (saved to local stream, not pushed yet)
(or) git commit -a (all)

## When you are done with your changes, you can push all your commits to the stream
git push origin <branch_name>               (all commits now show up on Github)

## If you want to revert back
git restore changed/changed_file            (revert back to original state)
git rm (-rf) changed/changed_file           (remove from git)

## To start your own branch, first make sure you're on main and it's up to date
git checkout main
git pull origin main

## Now can crreate and switch to your personal branch
git checkout -b <choose_your_branch_name>

## Push your branch to remote and set up tracking
git push -u origin <choose_your_branch_name> (makes sure you are pushing to your branch from now on)

## To switch between all available branches
git branch -a                               (list all available branches)
    
## Switch to an existing branch
git checkout <branch-name>

## Switch to main branch
git checkout main

## Create and switch to your branch
git checkout -b <choose_your_branch_name>
or
git checkout <choose_your_branch_name>

# Or create branch without switching
git branch new-branch-name

## Comparing your changes
git diff                                    (line by line changes)
git diff changed/changed_file               (changes in the specified file)
git diff --staged                           (changes from the files staged for commit)

git diff main..your-branch-name             (compare your branch with main)
git diff branch1..branch2                   (changes between two branches)
git diff main..your-branch-name -- filename (compare a specific file between branches)


## Commit history
git log
git log --oneline                   (see compact one-line history)
git log -p                          (see what changed in each commit)
git log --oneline -5                (see what changed in last 5 commits)

# Fetch changes without merging
git fetch

# Get changes and merge into current branch
git pull

# Update your branch with latest main changes
git checkout your-branch
git merge main

# Switch to main and get latest changes
git checkout main
git pull origin main

# Switch back to your branch and merge main into it (resolve conflicts if any)
git checkout your-branch-name
git merge main

# When ready, merge your branch into main
git checkout main
git merge your-branch-name
git push origin main
