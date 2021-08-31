fidCmcControls = fopen(strcat("Model_prescaled_adjusted_scaled-scaled_BodyKinematics_pos_global.sto"));
lineno = 1;
cnt = 1;
tline = fgetl(fidCmcControls);
while ischar(tline)
    lineno = lineno + 1;
    tline = fgetl(fidCmcControls);
    
    if lineno == 19
        data.headers = strsplit(tline,'\t');
    end
    if lineno > 19
        try
            data.values(cnt,:) = str2double(strsplit(tline,'\t'));
            cnt = cnt + 1;
        catch
%             disp("end of file")
        end
    end
end

calcn_r = data.values(1,32:34);
calcn_l = data.values(1,68:70);