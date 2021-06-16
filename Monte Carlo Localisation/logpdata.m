function loglikelihood = logpdata(z,zexp,beta)
    max_range_sensor=2;
    sigma=beta(3);
    out_of_range=1;
    for k = 1:size(z,1)
        if(z(k) < max_range_sensor)
            phit = pdf('Normal',z(k),zexp(k),sigma);
        else
            phit = 0;
        end
    prand = pdf('Uniform',z(k),0,out_of_range);
    p(k,1)=beta(1)*phit + beta(2)*prand;
   
    end
    
    plog=0;
    for k = 1:size(p,1)
        plog=plog+log(p(k,1));
    end
    loglikelihood=plog;
end