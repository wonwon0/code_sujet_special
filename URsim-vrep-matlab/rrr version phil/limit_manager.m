function AxeRes=limit_manager(numberLimitActive,LimitationActive,Axe)

        nbCross=0;              % Number of common lines between planes (number of 2 planes combinations) Might not be used.
        nbCrossOK=0;            % Number of common lines between planes that are not equals.
        Vectb=[];               % Resulting vector
        AxeRes=[0 0 0]';
        vect_ok=[];
        vectd_ok=[];
        vect_nok=[];

        
    if numberLimitActive==0 % If no active limitations  
        AxeRes=Axe; % The resulting vector is the initial vector
    elseif numberLimitActive==1 % If 1 active limitation
        Vecta=(Axe'*LimitationActive)*LimitationActive; % Component along restriceted direction
        AxeRes=Axe-Vecta; % Remove the component from the vector
    else %If several limit active
        for i=1:numberLimitActive % For all combination pairs between limitations planes
            for j=i+1:numberLimitActive
                nbCross=nbCross+1;%increment the number of intersections
                LimitationCross = cross(LimitationActive(:,i),LimitationActive(:,j));%Vector perpendicular to both planes
                if norm(LimitationCross)>0.0001 %If the planes are not parallel
                    nbCrossOK=nbCrossOK+1;%Increment the number of non-parallel combinations
                    LimitationCross=LimitationCross/norm(LimitationCross);%Normalize
                    Vectb(:,nbCrossOK)=(Axe'*LimitationCross)*LimitationCross;%Component of the vector along the common vector

                    % Vector in plane
                    Vectc1=cross(LimitationActive(:,i),LimitationCross); % Vector in plane 1, perpendicular to the common vector.
                    Vectc1=Vectc1/norm(Vectc1);

                    Vectc2=cross(LimitationActive(:,j),LimitationCross); % Vector in plane 2, perpendicular to the common vector.
                    Vectc2=Vectc2/norm(Vectc2);

                    Vectd1=(Axe'*Vectc1)*Vectc1; % Component in plane 1 along VectC1
                    Vectd2=(Axe'*Vectc2)*Vectc2; % Component in plane 2 along VectC2
                    Vectd1good=1;
                    Vectd2good=1;
                    if Vectd1'*LimitationActive(:,j)<0  % Verify if Vectd1 goes in plane 2
                        Vectd1good=0;
                    end
                    if Vectd2'*LimitationActive(:,i)<0  % Verify if Vectd2 goes in plane 1  
                        Vectd2good=0;
                    end

                    if Vectd1good == 1 % Only one of the two vector should be good. 
                        Vectd(:,nbCrossOK)=Vectd1;
                    elseif Vectd2good==1
                        Vectd(:,nbCrossOK)=Vectd2;
                    end
                end
            end
        end
        for i=1:size(Vectb,2)
            for j=1:size(LimitationActive,2)
                if Vectb(:,i)'*LimitationActive(:,j)<=0
                    vect_ok=[vect_ok, Vectb(:,i)];
                else
                    vect_nok=[vect_nok, Vectb(:,i)];
                end
            end
        end

        if exist('Vectd','var')
            for i=1:size(Vectd,2)
                for j=1:size(LimitationActive,2)
                    if Vectd'*LimitationActive(:,j)<0
                        vectd_ok=[vectd_ok, Vectd(:,i)];
                    end
                end
            end
        end
        for i=1:size(vect_ok,2)
            AxeRes=AxeRes+vect_ok(:,i);
        end
        for i=1:size(vectd_ok,2)
            AxeRes=AxeRes+vectd_ok(:,i);
        end
    %             for i=1:size(vect_nok,2)
    %                 d_min_nok(i)/limit_proximite
    %                 AxeRes=AxeRes+(d_min_nok(i)/limit_proximite)*vect_nok(:,i);
    %             end
        if AxeRes==([0 0 0]');
            ;
        elseif isempty(vect_ok)||size(vect_ok,2)>6
            AxeRes=([0 0 0]');
        else
            if norm(AxeRes)>1
                AxeRes=AxeRes/norm(AxeRes);
            else
                AxeRes=AxeRes;
            end
        end
    end
end