#include <stdio.h>
#include <math.h>
double EuclideanDistance(double sol[2], double goal_x, double goal_y)//计算某一点和终点的欧几里德距离
{
	double did;
	did = sqrt((sol[0] - goal_x) * (sol[0] - goal_x) + (sol[1] - goal_y) * (sol[1] - goal_y));
	return did;
}

double distance_to_line(double p[], double line_s[], double line_e[])
{
	double face;
	double opp_p ,opp_s ,opp_e;
	opp_p = EuclideanDistance(line_e, line_s[0], line_s[1]);
	opp_e = EuclideanDistance(p, line_s[0], line_s[1]);
	opp_s = EuclideanDistance(p, line_e[0], line_e[1]);
	if (( opp_p * opp_p + opp_e * opp_e - opp_s * opp_s) / (2 * opp_p * opp_e) <= 0)
		return opp_e;
	else if (( opp_p * opp_p + opp_s * opp_s - opp_e * opp_e) / (2 * opp_p * opp_s) <= 0)
		return opp_s;
    else{
		face =   fabs((p[0] - line_e[0]) * (line_s[1] - line_e[1]) - (p[1] - line_e[1]) * (line_s[0] - line_e[0]));
		return face / EuclideanDistance(line_e, line_s[0], line_s[1]);
	}
}
int main(){
	double p[2] = {-1,-3} , s[2] = {6,8} , e[2] = {6,8};
	double l;
	l = distance_to_line(p,s,e);
	printf("%lf",l);
}
