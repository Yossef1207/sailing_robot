truffle migrate --reset
cp /home/app/build/contracts/Election.json /home/app/src/contract/Election.json
node /home/app/src/Views/mailer/mailserver.js & npm start