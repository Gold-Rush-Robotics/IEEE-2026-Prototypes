# IEEE-2026-Prototypes

Repository for prototype code for the 2026 IEEE competition.

**Please publish any prototyping code you make to this repository!!**

## How to Contribute

This repository is the designated place for all prototype code created for the 2026 IEEE competition. To ensure the repository stays organized, please follow these guidelines when adding your work:

- **Create a branch** of the repository to make changes in and switch to it.
- **Create a dedicated folder** for your prototype. The folder name should be a clear, concise description of the task or project.
- **Add a `README.md` file inside your folder** that provides a brief overview of your code. This should include:
  - A short description of the prototype's purpose.
  - Basic instructions on how to use or run the prototype.

## How to clone the repository and push your code

If you have any issues with this, send a message on the GRR discord!
To get the repository on your local machine and to share your code, follow these steps:

1. **Clone the repository:**
   Open your terminal, cd into the folder you want to clone the repository into, and run this command:

    ```bash
    git clone https://github.com/Gold-Rush-Robotics/IEEE-2026-Prototypes.git
    ```

    If this fails, make sure you download git:
    - Windows: run `winget add git.git` in Terminal
    - Linux (debian-based): run `sudo apt update && sudo apt install git`
    - MacOS: see <https://git-scm.com/book/en/v2/Getting-Started-Installing-Git>

2. **Create a branch:**
   Create a branch for the feature you are working on. It should be a short but descriptive name (e.g. `servo-controller-logic`)

   ```bash
   # Create a new branch "new-branch-name" and switch to it
   git switch -c "new-branch-name"
   ```

3. **Add your files:**
    Place your new prototype folder and its contents into the local repository.

4. **Commit your changes:**

    ```bash
    git add .
    git commit -m "Replace this with a message about what you did"
    ```

5. **Publish your branch and push your commits so that they are available on the repository:**

    ```bash
    git push origin main
    ```

6. **Open a pull request:**
    When you are done with the code that you made, open a pull request to get your changes added to the main branch.

    1. Go to the [pull-request tab](https://github.com/Gold-Rush-Robotics/IEEE-2026-Prototypes/pulls).
    2. Click the "New" button in the top left.
    3. In the dropdown where it says "compare: main", change this to your branch. After this it should look something like `base: main <- compare: your-branch`. This means that you are taking the changes on the branch `your-branch` and adding them to the `main` branch (which is what we want).
    4. Click "Create Pull Request," add a title and description, then click "Create Pull Request" again.
    5. Wait for or get someone to review your PR. Once it has a passing review, you will be able to merge your pull request into the main branch!
