load 'sensordata.mat';
sensordatasub=sensordata(find((sensordata(:,2)>0.95) & (sensordata(:,2) < 1.05)),1);
histogram(sensordatasub);
z=sensordata(:,1);
zexp=sensordata(:,2);
beta = [0.6 0.4 0.5];
p= pmeasurement(z,zexp,beta);
loglikelihood = logpdata(z,zexp,beta);
%----17----
lb = [0 0 0];
ub = [1 1 5];
A = [];
b = [];
Aeq = [1 1 0];
beq = 1;
beta = [0.6 0.4 0.5];
% z=sensordata(find((sensordata(:,2)>0.95) & (sensordata(:,2) < 1.05)),1);
% zexp=sensordata(find((sensordata(:,2)>0.95) & (sensordata(:,2) < 1.05)),2);

optfun = @(x)(-logpdata(z,zexp,beta));
options=optimoptions('fmincon');
[betastar,fval] = fmincon(optfun,beta,A,b,Aeq,beq,lb,ub,[],options);
sensordatasub18=sensordata(find((sensordata(:,2)==1.0)),1);
histogram(sensordatasub18);