
# Creat an SSH key for passing to the docker container
For the container to be able to clone the private repository, you have to generate and ssh key and add it to your github account as explained [here](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent).

```sh
export SSH_PRIVATE_KEY="$(cat ~/.ssh/id_rsa)"
```