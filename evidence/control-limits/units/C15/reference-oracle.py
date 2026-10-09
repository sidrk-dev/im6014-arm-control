"""Parent exact certificate over emitted samples; independent of limiter code."""
from fractions import Fraction
import math

def finite_fraction(value):
    if type(value) is bool or not isinstance(value,(int,float,Fraction)):
        raise ValueError('Expected finite numeric sample')
    if isinstance(value,Fraction):return value
    if isinstance(value,float) and not math.isfinite(value):raise ValueError('Nonfinite sample')
    return Fraction(value)

def certify(*,previous_q,emitted_q,elapsed,wire_velocity,max_velocity,q_min,q_max,
            previous_step_velocity=0,previous_wire_velocity=0,
            max_acceleration=None,measured_q=None,max_lead=None):
    q0,q1,h,vw,V,lo,hi=map(finite_fraction,(previous_q,emitted_q,elapsed,wire_velocity,max_velocity,q_min,q_max))
    if h<=0 or V<=0 or lo>hi:raise ValueError('Invalid positive-step certificate inputs')
    step=(q1-q0)/h
    checks={'position':lo<=q1<=hi,'position_step_speed':abs(step)<=V,'encoded_speed':abs(vw)<=V}
    if max_acceleration is not None:
        A=finite_fraction(max_acceleration)
        if A<=0:raise ValueError('Invalid acceleration bound')
        checks['position_step_acceleration']=abs(step-finite_fraction(previous_step_velocity))<=A*h
        checks['encoded_acceleration']=abs(vw-finite_fraction(previous_wire_velocity))<=A*h
    if max_lead is not None:
        L=finite_fraction(max_lead);qm=finite_fraction(measured_q)
        if L<=0:raise ValueError('Invalid lead bound')
        checks['measured_lead']=abs(q1-qm)<=L
    return {'passed':all(checks.values()),'checks':checks,'position_step_velocity':step}
