#define URG_LEN  500   //384 

struct dataset {
	int	np;
	double	**p;
	int	nv;
	double	**v;
	int	skip;
	int	start;
	int	end;
	int	nref;
	int	**ref;
	int	type;
	int	ctype;
};

