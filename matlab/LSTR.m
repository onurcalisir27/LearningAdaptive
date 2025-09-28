classdef LSTR < handle
    properties
        lambda_;

    end

    methods
        function this = LSTR(forgetting_factor)
            this.lambda_ = forgetting_factor;
        end
        % Function to Return the Parameters and Covariance
        function [param,p] = getpar(this,theta,count,param,p,u)

            if count>2
                phi=[-theta(count-1) -theta(count-2) u(count-1) u(count-2)]';

                par=param(:,count-1);

                temp0=count-2;

                temp1=count-1;

                pt0 = p(:,4*temp0+1:4*temp0+4);

                [par1,pt1]=this.recls(par,pt0,phi,theta(count));

                p(:,4*temp1+1:4*temp1+4)=pt1;

                param(:,count)=par1;
            end

        end % Function End

        % Recursive Least Squares Update
        function [par1,p1] = recls(this, par0, p0, phi, y)

            temp=this.lambda_+phi'*p0*phi;

            k=p0*phi/temp;

            temp=y-par0'*phi;

            par1=par0+k*temp;

            temp=eye(4,4)-k*phi';

            p1=temp*p0/this.lambda_;

        end % Function End

        % Step Ahead Controller
        function [u2]=getu(this,par,u1,dy,y1,y2)

            chookh=1/par(3);

            t0=chookh;

            s0=par(1)*chookh;

            s1=par(2)*chookh;

            r1=par(4)*chookh;

            u2= t0*dy + s0*y2 + s1*y1 -r1*u1;

        end % Function end

        % Get Control Input
        function [torqv,pderr,u]=control(this, count,pderr,dtheta,theta,...
                                         param,u,gain)
            if count<3
                pderr(2)=pderr(1);
                pderr(1)=dtheta(count+1)-theta(count);

            else
                [te]=this.getu(param(:,count),u(count-1),dtheta(count+1),...
                         theta(count-1),theta(count));
                u(count)=te;
            end

            if count==1
                u(count)=gain(1)*pderr(1);
            end

            if count==2
                u(count)=gain'*pderr;
            end

            torqv=u(count);

        end % Function end

        % Physics Step Function
        function [dum]=funct(this,fun,dumx,dumthv,dumomv,torqv)

            gv=9.8;
            massv=0.1;
            lv=0.1;
            dampv=1;
            if fun==1
                dumthv=dumx;
            end
            if fun==2
                dumomv=dumx;
            end
            if fun==1
                dum=dumomv;
            end
            if fun==2
                 dum=(torqv-dampv*dumomv-massv*gv*lv*sin(dumthv))/(massv*lv^2);
            end

        end % Function end

        % Simulator Solver
        function [pderr,u,p,param,theta,omega,tn]=...
             solve(this,count,param,p,dtheta,theta,omega,tn,h1,u,pderr,...
             STEPS,gain)

            contac=1;
            ftemp=[0 0 0 0
                   0 0 0 0];

            if count>2
                % repet
                [param,p]=this.getpar(theta,count,param,p,u);
            end

            [torqv,pderr,u]=this.control(count,pderr,dtheta,theta,...
                            param,u,gain);

            rungthv=theta(count);
            rungomv=omega(count);
            xn=[rungthv rungomv]';
            dumx=xn;

            sar=1;
            while sar==1
                contac=contac+1;
                tn=tn+h1;

                if contac<=4
                    for fun=1:2
                        if fun==1
                            xn(fun)=rungthv;
                        elseif fun==2
                            xn(fun)=rungomv;
                        end

                        dumx(fun)=xn(fun);

                        [te]=this.funct(fun,dumx(fun),rungthv,rungomv,torqv);
                        ftemp(fun,contac-1)=te;
                        k1=h1*ftemp(fun,contac-1);

                        dumx(fun)=xn(fun)+k1/2.0;
                        [te]=this.funct(fun,dumx(fun),rungthv,rungomv,torqv);
                        k2=h1*te;

                        dumx(fun)=xn(fun)+k2/2.0;
                        [te]=this.funct(fun,dumx(fun),rungthv,rungomv,torqv);
                        k3=h1*te;

                        dumx(fun)=xn(fun)+k3;
                        [te]=this.funct(fun,dumx(fun),rungthv,rungomv,torqv);
                        k4=h1*te;
                        temp=(k1+2*k2+2*k3+k4)/6;
                        xn(fun)=xn(fun)+temp;
                    end

                    for fun=1:2
                        if contac<=STEPS+1
                            if fun==1
                                rungthv=xn(fun);
                            elseif fun==2
                                rungomv=xn(fun);
                            end
                        end
                    end
                end

                if contac>4
                    for fun=1:2
                        if fun==1
                            xn(fun)=rungthv;
                        elseif fun==2
                            xn(fun)=rungomv;
                        end

                        dumx(fun)=xn(fun);

                        [te]=this.funct(fun,dumx(fun),rungthv,rungomv,torqv);
                        ftemp(fun,4)=te;
                        xn(fun)=xn(fun)+h1*(55*ftemp(fun,4)-59*ftemp(fun,3)+...
                                 37*ftemp(fun,2)-9*ftemp(fun,1))/24;
                    end

                    for fun=1:2
                        if contac<=STEPS+1
                            if fun==1
                                rungthv=xn(fun);
                            end
                            if fun==2
                                rungomv=xn(fun);
                            end
                        end
                    end

                    for fun=1:2
                        if fun==1
                            xn(fun)=rungthv;
                        elseif fun==2
                            xn(fun)=rungomv;
                        end

                        for lop=1:3
                            ftemp(fun,lop)=ftemp(fun,lop+1);
                        end
                        [te]=this.funct(fun,dumx(fun),rungthv,rungomv,torqv);
                        ftemp(fun,4)=te;
                        dumx(fun)=dumx(fun)+h1*(9*ftemp(fun,4)+19*ftemp(fun,3)- ...
                                   5*ftemp(fun,2)+ftemp(fun,1))/24;
                        xn(fun)=dumx(fun);
                    end

                    for fun=1:2
                        if contac<=STEPS+1
                            if fun==1
                                rungthv=xn(fun);
                            elseif fun==2
                                rungomv=xn(fun);
                            end
                        end
                    end
                end

                if contac==STEPS+1
                    for fun=1:2
                        if fun==1
                            theta(count+1)=xn(fun);
                        elseif fun==2
                            omega(count+1)=xn(fun);
                        end
                        %    omv=omega'
                        %    tht=theta'
                    end

                    sar=0;
                end

                if contac<=STEPS
                    sar=1;
                end
            end

        end % Function end

    end % Methods end

end % Class end
