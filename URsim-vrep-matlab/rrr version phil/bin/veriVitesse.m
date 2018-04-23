function [v_output]=veriVitesse(v_input, d_min, pose_prox, pose_act)

limit_proximite=0.05;
v_input_save=v_input;
v_input_save2=v_input;
v_output=v_input;
v_output_save=v_input;
n=length(pose_prox(:,1));
for i=1:n
    if d_min(i)<limit_proximite
        if d_min(i)>0
            if (dot(pose_prox(i,:)-pose_act,v_input)>0.0000001)
            ratio_proximite(i)=1-d_min(i)/limit_proximite;
            vec_norm(i,:)=(pose_act-pose_prox(i,:))/norm(pose_act-pose_prox(i,:));
            %collision
            %définition de la vitesse tangeante à la surface
            v_output=v_input-ratio_proximite(i)*(v_input*vec_norm(i,:)')*vec_norm(i,:);
            %ajout de la composante normale à la surface qui varie linéairement
            %avec la distance restante à la surface
            end
            v_input=v_output;
        end
    end
end
%vérification de validité de la vitesse (backwrd verif)

for i=1:n
    if d_min(i)<limit_proximite
        if (dot(pose_prox(i,:)-pose_act,v_input_save)>0.0000001)
        %collision
        ratio_proximite(i)=1-d_min(i)/limit_proximite;
        %définition de la vitesse tangeante à la surface
        v_output_save=v_input_save-ratio_proximite(i)*(v_input_save*vec_norm(i,:)')*vec_norm(i,:);
        %ajout de la composante normale à la surface qui varie linéairement
        %avec la distance restante à la surface
        end
        v_input_save=v_output_save;
    end
end
    
%validation que v_input=v_input_save
%si v_output n'est pas le même résultat que v)ouput_save, deux
%limitations entrent en contradiction, il faut reclaculer.

if v_output_save~=v_output
    ratio_proximite=max(ratio_proximite);
    vec_norm=sum(vec_norm);
    vec_norm=vec_norm/norm(vec_norm);
    %on enlève la composante du vecteur vitesse quiest dans la direction conjointe des normales de plans 
    v_input=v_input_save2;
    if (dot(pose_prox(i,:)-pose_act,v_input)>0)
        %collision
        %définition de la vitesse tangeante à la surface
        v_output=v_input-ratio_proximite*(v_input*vec_norm')*vec_norm;
        %ajout de la composante normale à la surface qui varie linéairement
        %avec la distance restante à la surface
    end
end


if exist('vec_norm','var')
    sum(vec_norm);
end

dot(pose_prox(i,:)-pose_act,v_input);
v_output_save~=v_output



















