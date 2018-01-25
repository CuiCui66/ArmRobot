
class fraction{
    long num, den;
public:
    fraction(long num, long den):num(num), den(den){}
    fraction(long num):num(num), den(1){}
    fraction& operator+=(fraction f){
        num *= f.den;
        num += den * f.num;
        den*= f.den;
        return *this;
    }
    fraction operator-(){return fraction(-num,den);}
    fraction& operator-=(fraction f){return *this += (-f);}
    fraction& operator*=(fraction f){num*=f.num; den*=f.den;return *this;}
    fraction& operator/=(fraction f){num*=f.den; den*=f.num;return *this;}
#define OPERL(oper) fraction& operator oper(long l){return *this oper fraction(l,1);}
    OPERL(+=)
    OPERL(-=)
    OPERL(*=)
    OPERL(/=)

    operator long(){
        return num/den;
    }
};

#define OPERF(oper) fraction operator oper(fraction f, fraction f2){return f oper##= f2;}
OPERF(+);
OPERF(-);
OPERF(*);
OPERF(/);


#define OPERLL(oper) fraction operator oper(fraction f, long l){return f oper fraction(l,1);}
OPERLL(+);
OPERLL(-);
OPERLL(*);
OPERLL(/);

#define OPERLR(oper) fraction operator oper(long l, fraction f){return fraction(l,1) oper f;}
OPERLR(+);
OPERLR(-);
OPERLR(*);
OPERLR(/);

