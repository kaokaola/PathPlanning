/* ABC algorithm coded using C programming language */

/* Artificial Bee Colony (ABC) is one of the most recently defined algorithms by Dervis Karaboga in 2005,
motivated by the intelligent behavior of honey bees. */

/* Referance Papers*/

/*D. Karaboga, AN IDEA BASED ON HONEY BEE SWARM FOR NUMERICAL OPTIMIZATION,TECHNICAL REPORT-TR06, Erciyes University, Engineering Faculty, Computer Engineering Department 2005.*/

/*D. Karaboga, B. Basturk, A powerful and Efficient Algorithm for Numerical Function Optimization: Artificial Bee Colony (ABC) Algorithm, Journal of Global Optimization, Volume:39, Issue:3,pp:459-171, November 2007,ISSN:0925-5001 , doi: 10.1007/s10898-007-9149-x */

/*D. Karaboga, B. Basturk, On The Performance Of Artificial Bee Colony (ABC) Algorithm, Applied Soft Computing,Volume 8, Issue 1, January 2008, Pages 687-697. */

/*D. Karaboga, B. Akay, A Comparative Study of Artificial Bee Colony Algorithm,  Applied Mathematics and Computation, 214, 108-132, 2009. */

/*Copyright ? 2009 Erciyes University, Intelligent Systems Research Group, The Dept. of Computer Engineering*/

/*Contact:
Dervis Karaboga (karaboga@erciyes.edu.tr )
Bahriye Basturk Akay (bahriye@erciyes.edu.tr)
*/


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <conio.h>
#include <time.h>


/* Control Parameters of ABC algorithm*/
#define NP 100 /* The number of colony size (employed bees+onlooker bees)蜂群的大小，                                  为什么只包含两种蜜蜂？*/
#define FoodNumber NP/2 /*The number of food sources equals the half of the colony size                               为什么要是一般大小*/
#define limit 100  /*A food source which could not be improved through "limit" trials is abandoned by its employed bee一个蜜源在未能改进的情况下最多访问次数*/
#define maxCycle 1000 /*The number of cycles for foraging {a stopping criteria}                                       算法的执行循环次数*/

/* Problem specific variables*/
#define D 20 /*The number of parameters of the problem to be optimized                                                处理对象的描述参数，此处为50个*/
#define lb -5.12 /*lower bound of the parameters.                                                                     描述参数的上下界，此处50个参数采用同样的上下界，可以扩增*/
#define ub 5.12 /*upper bound of the parameters. lb and ub can be defined as arrays for the problems of which parameters have different bounds*/

#define NumberOfArrayB 10																							/*定义描述参数的上下界组数用于针对特定问题*/
#define runtime 30  /*Algorithm can be run many times in order to see its robustness                                  定义总的循环次数，取平均看效率*/
#define map_point 12
#define M_PI       3.14159265358979323846   // pi																	  已经是最大精度了

#define collision 1000*1000																							 /*此处为冲撞状态的权值，应该要保证比较大*/
#define length    1																									 /*此处为长度的权值*/

double k;
int court;
double Foods[FoodNumber][D]; /*Foods is the population of food sources. Each row of Foods matrix is a vector holding D parameters to be optimized. The number of rows of Foods matrix equals to the FoodNumber*/
double f[FoodNumber];  /*f is a vector holding objective function values associated with food sources                 各食物源关于目标函数的属性 */
double fitness[FoodNumber]; /*fitness is a vector holding fitness (quality) values associated with food sources       各食物源关于目标函数的适应性*/
double trial[FoodNumber]; /*trial is a vector holding trial numbers through which solutions can not be improved       各食物源未被优化是的访问次数*/
double prob[FoodNumber]; /*prob is a vector holding probabilities of food sources (solutions) to be chosen            各食物源被跟随蜂选中的概率*/
double solution[D]; /*New solution (neighbour) produced by v_{ij}=x_{ij}+\phi_{ij}*(x_{kj}-x_{ij}) j is a randomly chosen parameter and k is a randomlu chosen solution different from i依据随机策略生成的新的食物源*/

double map[map_point][2]={{0,0},{100,0},{100,100},{0,100},{40,20},{60,20},{60,40},{40,40},{40,50},{60,50},{60,100},{40,100}};	//存储地图信息，由数个节点绘成，先画个主观地图
double StartandGoal[2][D] = { {0,0,20,90}, {0,0,85,70} };																							//存储起点和终点
double Best[20][D];																									//用于存储选取到的最佳点

double ObjValSol; /*Objective function value of new solution														  新食物源的f*/
double FitnessSol; /*Fitness value of new solution																	  新食物源的fitness*/
int neighbour, param2change; /*param2change corrresponds to j, neighbour corresponds to k in equation v_{ij}=x_{ij}+\phi_{ij}*(x_{kj}-x_{ij})相当于k和j*/
double GlobalMin; /*Optimum solution obtained by ABC algorithm                                                        某一全局最小值*/
double GlobalParams[D]; /*Parameters of the optimum solution                                                          某一全局最小值的描述参数*/
double GlobalMins[runtime]; /*GlobalMins holds the GlobalMin of each run in multiple runs                             全部全局最小值*/
double r; /*a random number in the range [0,1)                                                                        用于存储每次产生的随机数，此处要注意随机种子的更新*/

		  /*a function pointer returning double and taking a D-dimensional array as argument */
		  /*If your function takes additional arguments then change function pointer definition and lines calling "...=function(solution);" in the code*/
typedef double(*FunctionCallback)(double sol[D]);

/*benchmark functions */
double sphere(double sol[D]);
double Rosenbrock(double sol[D]);
double Griewank(double sol[D]);
double Rastrigin(double sol[D]);
double Pathplan(double sol[D]);

/*Write your own objective function name instead of sphere*/
FunctionCallback function = &Rastrigin;                                         //函数选择

/*需要先刻画地图范围map[map_point]*/
/*我要写个障碍安全判断函数linecollidefree, pointcollidefree*/
/*增加一个距离计算函数EuclideanDistance*/
/*增加一个数组存贮每一轮的节点信息和起点终点信息Best和startandgoal*/
/*增加一个针对路径规划的综合评价函数PathFunction,而且要有对应的权值存储*/


double EuclideanDistance(double sol[D], double goal_x, double goal_y)//计算某一点和终点的欧几里德距离
{
	double dis;
	dis = sqrt((sol[3] - goal_x) * (sol[3] - goal_x) + (sol[4] - goal_y) * (sol[4] - goal_y));
	return dis;
}

int Pointcollidefree(double x, double y)//判断某一点是否在障碍区，若不在，返回0，若在，返回1
{
	int result = 0;
	if (x <= 0 || y <= 0 || x >= 100 || y >= 100)
	{
		printf("wrong nodes!!!!");
		return 1;
	}
	else if ((x >= 40 && x <= 60) && (y >= 20 && y <= 40 || y >= 60 && y <= 100))
		return 1;
	else
		return 0;
}

int linecollidefree(double sol[D], int last)//判断某一点和上一个最佳点之间是否有障碍。last为上一点的索引。1有障碍，0无障碍
{
	int number, j;
	int danweimidu = 1;
	double distance, distance_x, distance_y, unit_x, unit_y, x, y;

	distance = EuclideanDistance(sol, Best[last][3], Best[last][4]);									//此处可能会有问题，注意
	number = (int)(distance / danweimidu);

	distance_x = Best[last][3] - sol[3];
	distance_y = Best[last][4] - sol[4];
	unit_x = distance_x / number;
	unit_y = distance_y / number;
	x = Best[last][3];
	y = Best[last][4];
	for (j = 1; j < number; j++)
	{
		x = x + unit_x;
		y = y + unit_y;
		if (Pointcollidefree(x, y))
			return 1;
	}
	return 0;
}


double PathFunction(double sol[D])//此处属性有四个，一是是否冲撞，二是距离。三、四为目标的坐标。按顺序存储
{
	int j = 0;
	double top = 0;
	if (j == 0)//判断是否冲撞,值为一或者零
	{
		top = top + sol[j] * collision;
		j++;
	}
	else if (j == 1)//对长度加权，值为一浮点数
	{
		top = top + sol[j] * length;
	}
	else
		top = top;
	return top;
}
																														/*Fitness function*/
double CalculateFitness(double fun)
{
	double result = 0;
	if (fun >= 0)
	{
		result = 1 / (fun + 1);//为什么要选这种计算方式？
	}
	else
	{
		result = 1 + fabs(fun);//fabs计算浮点数的绝对值
	}
	return result;
}

/*The best food source is memorized*/
void MemorizeBestSource()
{
	int i, j;

	for (i = 0; i<FoodNumber; i++)
	{ 
		if (f[i]<GlobalMin)
		{
			GlobalMin = f[i];
			for (j = 0; j<D; j++)
				GlobalParams[j] = Foods[i][j];
		}
	}
}

/*Variables are initialized in the range [lb,ub]. If each parameter has different range, use arrays lb[j], ub[j] instead of lb and ub */
/* Counters of food sources are also initialized in this function*/
void init(int index)
{
	int j;
	for (j = 0; j<D; j++)
	{
		r = ((double)rand() / ((double)(RAND_MAX)+(double)(1)));
		Foods[index][j] = r*(ub - lb) + lb;
		solution[j] = Foods[index][j];
	}
	f[index] = function(solution);
	fitness[index] = CalculateFitness(f[index]);
	trial[index] = 0;
}

/*All food sources are initialized */
void initial()
{
	int i;
	for (i = 0; i<FoodNumber; i++)
	{
		init(i);
	}
	GlobalMin = f[0];
	for (i = 0; i<D; i++)
		GlobalParams[i] = Foods[0][i];


}

void SendEmployedBees()
{
	int i, j;
	/*Employed Bee Phase*/
	for (i = 0; i<FoodNumber; i++)//依次对所有蜜源进行变异、贪婪选择处理
	{
		/*The parameter to be changed is determined randomly*/
		r = ((double)rand() / ((double)(RAND_MAX)+(double)(1)));
		param2change = (int)(r*D);								//0-49都能够访问到，而且概率相等

																/*A randomly chosen solution is used in producing a mutant solution of the solution i*/
		r = ((double)rand() / ((double)(RAND_MAX)+(double)(1)));
		neighbour = (int)(r*FoodNumber);

		/*Randomly selected solution must be different from the solution i*/
		while (neighbour == i)
		{
			r = ((double)rand() / ((double)(RAND_MAX)+(double)(1)));
			neighbour = (int)(r*FoodNumber);
		}
		for (j = 0; j<D; j++)
			solution[j] = Foods[i][j];

		/*v_{ij}=x_{ij}+\phi_{ij}*(x_{kj}-x_{ij}) */
		r = ((double)rand() / ((double)(RAND_MAX)+(double)(1)));
		solution[param2change] = Foods[i][param2change] + (Foods[i][param2change] - Foods[neighbour][param2change])*(r - 0.5) * 2;//只针对50中的一个参数变异

		/*if generated parameter value is out of boundaries, it is shifted onto the boundaries*/
		if (solution[param2change]<lb)
			solution[param2change] = lb;
		if (solution[param2change]>ub)
			solution[param2change] = ub;
		ObjValSol = function(solution);//结合所有属性信息获得一个综合评价值
		FitnessSol = CalculateFitness(ObjValSol);//根据属性计算适应度

		 /*a greedy selection is applied between the current solution i and its mutant*/
		if (FitnessSol>fitness[i])
		{
			/*If the mutant solution is better than the current solution i, replace the solution with the mutant and reset the trial counter of solution i*/
			trial[i] = 0;//选择新的蜜源，访问次数置零
			for (j = 0; j<D; j++)
				Foods[i][j] = solution[j];
			f[i] = ObjValSol;
			fitness[i] = FitnessSol;}
		else
		{   
			/*if the solution i can not be improved, increase its trial counter*/
			trial[i] = trial[i] + 1;}


	}

	/*end of employed bee phase*/

}

/* A food source is chosen with the probability which is proportioal to its quality*/
/*Different schemes can be used to calculate the probability values*/
/*For example prob(i)=fitness(i)/sum(fitness)*/
/*or in a way used in the metot below prob(i)=a*fitness(i)/max(fitness)+b*/
/*probability values are calculated by using fitness values and normalized by dividing maximum fitness value*/
void CalculateProbabilities()//基于蜜源的质量计算优先概率
{
	int i;
	double maxfit;
	maxfit = fitness[0];
	for (i = 1; i<FoodNumber; i++)//获取已存在的最高适应度
	{
		if (fitness[i]>maxfit)
			maxfit = fitness[i];
	}

	for (i = 0; i<FoodNumber; i++)
	{
		prob[i] = (0.9*(fitness[i] / maxfit)) + 0.1;//把所有适应度放到0.1-1中去
	}

}

void SendOnlookerBees()//进行跟随蜜蜂的计算过程。
{

	int i, j, t;
	i = 0;
	t = 0;
	/*onlooker Bee Phase*/
	while (t<FoodNumber)//对所有的食物源进行概率性更新
	{

		r = ((double)rand() / ((double)(RAND_MAX)+(double)(1)));
		if (r<prob[i]) /*choose a food source depending on its probability to be chosen*/
		//只有优先级大于随机概率的蜜源才会被跟随蜜蜂接受。随机值是每执行一次都会改变的。
		{
			t++;
			//跟随蜜蜂选中蜜源之后便成为雇佣蜂，进行类似操作。
			/*The parameter to be changed is determined randomly*/
			r = ((double)rand() / ((double)(RAND_MAX)+(double)(1)));
			param2change = (int)(r*D);//j

			/*A randomly chosen solution is used in producing a mutant solution of the solution i*/
			r = ((double)rand() / ((double)(RAND_MAX)+(double)(1)));
			neighbour = (int)(r*FoodNumber);//k

			/*Randomly selected solution must be different from the solution i*/
			while (neighbour == i)
			{
				r = ((double)rand() / ((double)(RAND_MAX)+(double)(1)));
				neighbour = (int)(r*FoodNumber);
			}
			for (j = 0; j<D; j++)
				solution[j] = Foods[i][j];

			/*v_{ij}=x_{ij}+\phi_{ij}*(x_{kj}-x_{ij}) */
			r = ((double)rand() / ((double)(RAND_MAX)+(double)(1)));
			solution[param2change] = Foods[i][param2change] + (Foods[i][param2change] - Foods[neighbour][param2change])*(r - 0.5) * 2;

			/*if generated parameter value is out of boundaries, it is shifted onto the boundaries*/
			if (solution[param2change]<lb)
				solution[param2change] = lb;
			if (solution[param2change]>ub)
				solution[param2change] = ub;
			ObjValSol = function(solution);
			FitnessSol = CalculateFitness(ObjValSol);

			/*a greedy selection is applied between the current solution i and its mutant*/
			if (FitnessSol>fitness[i])
			{
				/*If the mutant solution is better than the current solution i, replace the solution with the mutant and reset the trial counter of solution i*/
				trial[i] = 0;
				for (j = 0; j<D; j++)
					Foods[i][j] = solution[j];
				f[i] = ObjValSol;
				fitness[i] = FitnessSol;
			}
			else
			{   /*if the solution i can not be improved, increase its trial counter*/
				trial[i] = trial[i] + 1;
			}
		} /*if */
		i++;
		if (i == FoodNumber)
			i = 0;					//t只有被选中才会自增，而i每次都会自增，二者不同步。
	}/*while*/

	 /*end of onlooker bee phase     */
}
/*determine the food sources whose trial counter exceeds the "limit" value. In Basic ABC, only one scout is allowed to occur in each cycle*/
void SendScoutBees()//判断是否要放弃蜜源
{
	int maxtrialindex, i;
	maxtrialindex = 0;
	for (i = 1; i<FoodNumber; i++)					//得出最大的访问次数的index
	{
		if (trial[i]>trial[maxtrialindex])
			maxtrialindex = i;
	}
	if (trial[maxtrialindex] >= limit)				//判断是否对最大访问次数的蜜源执行放弃操作，然后初始化。
	{
		init(maxtrialindex);
	}
}


/*Main program of the ABC algorithm*/
int main()
{

	clock_t start, finish;						    //记录时间
	double totaltime;
	start = clock();


	int iter, run, j;
	double mean;
	for(k = 0.5 ;k <= 0.9 ;k += 0.02){
		printf("k = %12f\n",k);
		mean = 0;//平均值
		for (run = 0; run < runtime; run++)
		{
			court = 0;
			srand(time(NULL));								//以当前系统时间获取随机种子的操作
			initial();										//随机初始化所有蜜源
			MemorizeBestSource();							//初始化最佳蜜源
			for (iter = 0; iter<maxCycle; iter++)
			{
				court = iter;
				SendEmployedBees();
				CalculateProbabilities();
				SendOnlookerBees();
				MemorizeBestSource();
				SendScoutBees();
			}
			for (j = 0; j<D; j++)
			{
				//printf("GlobalParam[%d]: %f\n", j + 1, GlobalParams[j]);
			}
			//printf("%d. run: %e \n", run + 1, GlobalMin);
			GlobalMins[run] = GlobalMin;
			mean = mean + GlobalMin;
		}
		mean = mean / runtime;	
		printf("Means of %d runs: %e\n\n", runtime, mean);
	}
	finish = clock();
	totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
	printf("the total time is : %f\n", totaltime);
	_getch();   //i have changed the code here, so  i may change function attention????!!!!!!!!!!!!!!!!!!!!!!!!!!
}


double sphere(double sol[D])
{
	int j;
	double top = 0;
	for (j = 0; j<D; j++)
	{
		top = top + sol[j] * sol[j];
	}
	return top;
}

double Rosenbrock(double sol[D])
{
	int j;
	double top = 0;
	for (j = 0; j<D - 1; j++)
	{
		top = top + 100 * pow((sol[j + 1] - pow((sol[j]), (double)2)), (double)2) + pow((sol[j] - 1), (double)2);
	}
	return top;
}

double Griewank(double sol[D])
{
	int j;
	double top1, top2, top;
	top = 0;
	top1 = 0;
	top2 = 1;
	for (j = 0; j<D; j++)
	{
		top1 = top1 + pow((sol[j]), (double)2);
		top2 = top2*cos((((sol[j]) / sqrt((double)(j + 1)))*M_PI) / 180);

	}
	top = (1 / (double)4000)*top1 - top2 + 1;
	return top;
}

double Rastrigin(double sol[D])
{
	int j;
	double top = 0 ,p ,fl;
	srand(time(NULL));
	p = ((double)rand() / ((double)(RAND_MAX)+(double)(1)));
	fl = k * p *(1/(maxCycle + 1 -court));
	for (j = 0; j < D; j++)
	{
		top = top + (pow(sol[j], (double)2) - 10 * cos(2 * M_PI * sol[j]) + 10);
	}
	top += top * fl;
	return top;
}
