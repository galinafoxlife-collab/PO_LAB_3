FUNCTION_BLOCK FB_Integrator (* Discrete integrator *)
    VAR_INPUT
        in : REAL;
    END_VAR
    VAR_OUTPUT
        out : REAL;
    END_VAR
    VAR
        dt : REAL;
    END_VAR
END_FUNCTION_BLOCK

FUNCTION_BLOCK FB_Motor (* DC motor model used by the simulation *)
    VAR_INPUT
        u : REAL;
    END_VAR
    VAR_OUTPUT
        w : REAL;
        phi : REAL;
    END_VAR
    VAR
        Tm : REAL;
        ke : REAL;
        dt : REAL;
    END_VAR
END_FUNCTION_BLOCK

FUNCTION_BLOCK FB_Regulator (* Speed PI with anti-windup *)
    VAR_INPUT
        e : REAL;
        enabled : BOOL;
    END_VAR
    VAR_OUTPUT
        u : REAL;
    END_VAR
    VAR
        k_p : REAL;
        k_i : REAL;
        integrator : FB_Integrator;
        max_abs_value : REAL;
        dt : REAL;
    END_VAR
END_FUNCTION_BLOCK

FUNCTION_BLOCK FB_Axis (* Encoder speed and signed motor PWM *)
    VAR_INPUT
        reset_error : BOOL;
        reset_counter_request : BOOL;
        endswitch_a_reached : BOOL;
        endswitch_b_reached : BOOL;
        encoder_ok : BOOL;
        drive_enabled : BOOL;
        counter : INT;
        u : REAL;
    END_VAR
    VAR_OUTPUT
        reset_counter : BOOL;
        pwm_value : INT;
        speed : REAL;
    END_VAR
    VAR
        last_counter : INT;
        counter_valid : BOOL;
    END_VAR
END_FUNCTION_BLOCK
