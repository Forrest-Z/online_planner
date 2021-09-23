function [thrust, rpy_rate] = compute_control(m, g, curr_state, target_state)
    p = curr_state.p;
    v = curr_state.v;
    R = curr_state.a;
    
    p_d = target_state.p;
    v_d = target_state.v;
    a_d = target_state.a;
    
end

