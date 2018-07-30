## copy file when ssh
[link](https://unix.stackexchange.com/questions/106480/how-to-copy-files-from-one-machine-to-another-using-ssh)
To copy a file from `B` to `A` while logged into `B`: <br/>
`$ scp /path/to/file username@a:/path/to/destination` <br/>
To copy a file from B to A while logged into A: <br/>
`$ scp username@b:/path/to/file /path/to/destination` <br/>
