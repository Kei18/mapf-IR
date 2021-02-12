#!/bin/bash
# msg -> slack, use Incoming WebHook

MESSAGE=$1
HOST=`hostname`

if [ ${#SLACK_URL} -eq 0 ]
then
    echo "please setup SLACK_URL"
    exit 0;
fi

payload="payload={
  \"username\": \"${HOST}\",
  \"text\": \"${MESSAGE}\",
  \"icon_emoji\": \":heavy_check_mark:\"
}"

curl -s -S -X POST --data-urlencode "${payload}" ${SLACK_URL} > /dev/null
