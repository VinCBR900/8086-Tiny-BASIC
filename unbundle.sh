# execute to unbundle tools 

# fix windows CRLF issues
tr -d '\r' < /mnt/user-data/uploads/bundle.txt | base64 -d > bundle.tar.gz
tar xzf bundle.tar.gz -C /home/claude/tools
cd /home/claude/tools && make