function roots = solveQuartic( factors )

    A = factors(1,1);
    B = factors(1,2);
    C = factors(1,3);
    D = factors(1,4);
    E = factors(1,5);
    
    A_pw2 = A*A;
    B_pw2 = B*B;
    A_pw3 = A_pw2*A;
    B_pw3 = B_pw2*B;
    A_pw4 = A_pw3*A;
    B_pw4 = B_pw3*B;
    
    alpha = -3*B_pw2/(8*A_pw2)+C/A;
    beta = B_pw3/(8*A_pw3)-B*C/(2*A_pw2)+D/A;
    gamma = -3*B_pw4/(256*A_pw4)+B_pw2*C/(16*A_pw3)-B*D/(4*A_pw2)+E/A;
    
    alpha_pw2 = alpha*alpha;
    alpha_pw3 = alpha_pw2*alpha;
    
    P = -alpha_pw2/12-gamma;
    Q = -alpha_pw3/108+alpha*gamma/3-beta^2/8;
    R = -Q/2+sqrt(Q^2/4+P^3/27);
    U = R^(1/3);
    
    if U == 0
        y = -5*alpha/6-Q^(1/3);
    else
        y = -5*alpha/6-P/(3*U)+U;
    end
    
    w = sqrt(alpha+2*y);
    
    roots(1,1) = -B/(4*A) + 0.5*(w+sqrt(-(3*alpha+2*y+2*beta/w)));
    roots(2,1) = -B/(4*A) + 0.5*(w-sqrt(-(3*alpha+2*y+2*beta/w)));
    roots(3,1) = -B/(4*A) + 0.5*(-w+sqrt(-(3*alpha+2*y-2*beta/w)));
    roots(4,1) = -B/(4*A) + 0.5*(-w-sqrt(-(3*alpha+2*y-2*beta/w)));
    
end