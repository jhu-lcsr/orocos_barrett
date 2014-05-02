#include <math.h>
#include <oro_barrett_hw/butterworth_solver.h>

/* This butterworth filter design implementation taken shamelessly from
 * the ELLF Digital Filter Calculator
 * 
 * Originally written by Stephen L. Moshier, December 1986
 * Last revision: November, 1992
 *
 * http://www.moshier.net/ellfdoc.html
 * 
 * Math routines from the Cephes Math Library,
 * Release 2.1 January 1989 Copyright 1984, 1987, 1989 Stephen L. Moshier
 * Release 2.8 June    2000 Copyright 1984, 1995, 2000 Stephen L. Moshier
 * Direct inquiries to 30 Frost Street, Cambridge, MA 02140
 */

#define PREC 27
#define MAXEXP 1024
#define MINEXP -1077

/* size of arrays: */
#define ARRSIZ 1024

#define PI (3.1415926535897932384626433)

typedef struct
{
	double r;
	double i;
} cmplx;

static cmplx cone = { 1.0, 0.0 };

/*	c = b + a	*/
static void comp_add(cmplx *a, cmplx *b, cmplx *c)
{
	c->r = b->r + a->r;
	c->i = b->i + a->i;
}

/*	c = b - a	*/
static void comp_sub(cmplx *a, cmplx *b, cmplx *c)
{
	c->r = b->r - a->r;
	c->i = b->i - a->i;
}

/*	c = b * a */
static void comp_mul(cmplx *a, cmplx *b, cmplx *c)
{
	double y;
	y = b->r * a->r - b->i * a->i;
	c->i = b->r * a->i + b->i * a->r;
	c->r = y;
}

/*	c = b / a
 * returns -1 on overflow */
static int comp_div(cmplx *a, cmplx *b, cmplx *c)
{
	double y, p, q, w;
	y = a->r * a->r + a->i * a->i;
	p = b->r * a->r + b->i * a->i;
	q = b->i * a->r - b->r * a->i;
	if (y < 1.0)
	{
		w = HUGE_VAL * y;
		if ((fabs(p) > w) || (fabs(q) > w) || (y == 0.0))
		{
			c->r = HUGE_VAL;
			c->i = HUGE_VAL;
			return -1;
		}
	}
	c->r = p / y;
	c->i = q / y;
   return 0;
}

/*	b = a; Caution, a `short' is assumed to be 16 bits wide.  */
static void cmov(void *a, void *b)
{
	short *pa, *pb;
	int i;
	pa = (short *)a;
	pb = (short *)b;
	i = 8;
	do
		*pb++ = *pa++;
	while (--i);
}

static double comp_abs(cmplx *z)
{
	double x, y, b, re, im;
	int ex, ey, e;

	re = fabs(z->r);
	im = fabs(z->i);

	if (re == 0.0)
		return (im);
	if (im == 0.0)
		return (re);

   /* Get the exponents of the numbers */
	x = frexp(re, &ex);
	y = frexp(im, &ey);

   /* Check if one number is tiny compared to the other */
	e = ex - ey;
	if (e > PREC)
		return (re);
	if (e < -PREC)
		return (im);

   /* Find approximate exponent e of the geometric mean. */
	e = (ex + ey) >> 1;

   /* Rescale so mean is about 1 */
	x = ldexp(re, -e);
	y = ldexp(im, -e);

   /* Hypotenuse of the right triangle */
	b = sqrt(x * x + y * y);

   /* Compute the exponent of the answer. */
	y = frexp(b, &ey);
	ey = e + ey;

   /* Check it for overflow and underflow. */
	if (ey > MAXEXP)
	{
		return HUGE_VAL;
	}
	if (ey < MINEXP)
		return (0.0);

   /* Undo the scaling */
	b = ldexp(b, e);
	return (b);
}

static int comp_sqrt(cmplx *z, cmplx *w)
{
	int err;
   cmplx q, s;
	double x, y, r, t;

	x = z->r;
	y = z->i;

	if (y == 0.0)
	{
		if (x < 0.0)
		{
			w->r = 0.0;
			w->i = sqrt(-x);
			return 0;
		}
		else
		{
			w->r = sqrt(x);
			w->i = 0.0;
			return 0;
		}
	}

	if (x == 0.0)
	{
		r = fabs(y);
		r = sqrt(0.5 * r);
		if (y > 0)
			w->r = r;
		else
			w->r = -r;
		w->i = r;
		return 0;
	}

   /* Approximate  sqrt(x^2+y^2) - x  =  y^2/2x - y^4/24x^3 + ... .
    * The relative error in the first term is approximately y^2/12x^2 .
    */
	if ((fabs(y) < 2.e-4 * fabs(x)) && (x > 0))
	{
		t = 0.25 * y * (y / x);
	}
	else
	{
		r = comp_abs(z);
      if (r == HUGE_VAL)
         return -1;
		t = 0.5 * (r - x);
	}

	r = sqrt(t);
	q.i = r;
	q.r = y / (2.0 * r);
   /* Heron iteration in complex arithmetic */
	err = comp_div(&q, z, &s);
   if (err) return err;
	comp_add(&q, &s, w);
	w->r *= 0.5;
	w->i *= 0.5;
   return 0;
}

int butterworth_solver(const int type, const int order,
   const double fs, const double fa, const double fb,
   double * as, double * bs)
{
   int err;
   double aa[ARRSIZ];
   double pp[ARRSIZ];
   double y[ARRSIZ];
   double zs[ARRSIZ];
   cmplx z[ARRSIZ];
   double f1;
   double f2;
   double cbp = 0.0;
   double wc = 0.0;
   double c = 0.0;
   double cgam = 0.0;
   double scale = 0.0;
   double a = 0.0;
   double b = 0.0;
   double m = 0.0;
   double cang = 0.0;
   double bw = 0.0;
   double ang = 0.0;
   double fnyq = 0.0;
   double ai = 0.0;
   double pn = 0.0;
   double an = 0.0;
   double gam = 0.0;
   double cng = 0.0;
   double gain = 0.0;
   int lr = 0;
   int i = 0;
   int j = 0;
   int jt = 0;
   int nc = 0;
   int ii = 0;
   int ir = 0;
   int zord = 0;
   int zord_expected;
   int icnt = 0;
   int mh = 0;
   int jj = 0;
   int jh = 0;
   int jl = 0;
   int np = 0;
   int nz = 0;
   cmplx r, cnum, cden, cwc, ca, cb, b4ac;
   double C;
   cmplx lin[2];
   
   /* Zero things */
   for (i=0; i<ARRSIZ; i++)
   {
      zs[i] = 0.0;
      z[i].r = 0.0;
      z[i].i = 0.0;
   }
 
	fnyq = 0.5 * fs;
   
   switch (type)
   {
   case BUTTER_LOWPASS:
      bw = fa;
      a = fa;
      f1 = 0.0;
      f2 = fa;
      break;
   case BUTTER_HIGHPASS:
      bw = fa;
      a = fnyq;
      f2 = fa;
      f1 = 0.0;
      break;
   case BUTTER_BANDPASS:
   case BUTTER_BANDSTOP:
      bw = fb - fa;
      a = fb;
      f2 = fb;
      f1 = fa;
      break;
   }
   
   /* Frequency correspondence for bilinear transformation
    *    Wanalog = tan( 2 pi Fdigital T / 2 )
    * where T = 1/fs
    */
	ang = bw * PI / fs;
	cang = cos(ang);
	c = sin(ang) / cang;	/* Wanalog */
	wc = c;

   /* Transformation from low-pass to band-pass critical frequencies
    *
    * Center frequency
    *                     cos( 1/2 (Whigh+Wlow) T )
    *  cos( Wcenter T ) = ----------------------
    *                     cos( 1/2 (Whigh-Wlow) T )
    *
    *
    * Band edges
    *            cos( Wcenter T) - cos( Wdigital T )
    *  Wanalog = -----------------------------------
    *                        sin( Wdigital T )
    */
   
   /* Butterworth */
   a = PI * (a + f1) / fs;
   cgam = cos(a) / cang;
   a = 2.0 * PI * f2 / fs;
   cbp = (cgam - cos(a)) / sin(a);
   scale = 1.0;

   /* calculate s plane poles and zeros, normalized to wc = 1 */
   np = (order + 1) / 2;
   nz = 0;
   /* Butterworth poles equally spaced around the unit circle */
   m = (order & 1) ? 0.0 : PI/(2.0*order);
   for (i = 0; i < np; i++)
   {								  /* poles */
      lr = i + i;
      zs[lr] = -cos(m);
      zs[lr + 1] = sin(m);
      m += PI / order;
   }
   /* high pass or band reject */
   if (type == BUTTER_HIGHPASS || type == BUTTER_BANDSTOP)
   {
      /* map s => 1/s */
      for (j=0; j < np; j++)
      {
         ir = j + j;
         ii = ir + 1;
         b = zs[ir] * zs[ir] + zs[ii] * zs[ii];
         zs[ir] = zs[ir] / b;
         zs[ii] = zs[ii] / b;
      }
      /* The zeros at infinity map to the origin */
      nz = np;
      if (type == BUTTER_BANDSTOP)
      {
         nz += order / 2;
      }
      for (j = 0; j < nz; j++)
      {
         ir = ii + 1;
         ii = ir + 1;
         zs[ir] = 0.0;
         zs[ii] = 0.0;
      }
   }

   /* convert s plane to z plane */
   C = wc;
   nc = np;
   jt = -1;
   ii = -1;
   for (icnt=0; icnt < 2; icnt++)
   {
      /* The maps from s plane to z plane */
      do
      {
         ir = ii + 1;
         ii = ir + 1;
         r.r = zs[ir];
         r.i = zs[ii];

         switch (type)
         {
         case BUTTER_LOWPASS:
         case BUTTER_HIGHPASS:
            /* Substitute  s - r  =  s/wc - r = (1/wc)(z-1)/(z+1) - r
             *
             *     1  1 - r wc (       1 + r wc )
             * =  --- -------- ( z  -  -------- )
             *    z+1    wc    (       1 - r wc )
             *
             * giving the root in the z plane.
             */
            cnum.r = 1 + C * r.r;
            cnum.i = C * r.i;
            cden.r = 1 - C * r.r;
            cden.i = -C * r.i;
            jt += 1;
            err = comp_div(&cden, &cnum, &z[jt]);
            if (err) return err;
            if (r.i != 0.0)
            {
               /* fill in complex conjugate root */
               jt += 1;
               z[jt].r = z[jt - 1].r;
               z[jt].i = -z[jt - 1].i;
            }
            break;
         case BUTTER_BANDPASS:
         case BUTTER_BANDSTOP:
            /* Substitute  s - r  =>  s/wc - r
             *
             *     z^2 - 2 z cgam + 1
             * =>  ------------------  -  r
             *         (z^2 + 1) wc  
             *
             *         1
             * =  ------------  [ (1 - r wc) z^2  - 2 cgam z  +  1 + r wc ]
             *    (z^2 + 1) wc  
             *
             * and solve for the roots in the z plane.
             */
            cwc.r = c;
            cwc.i = 0.0;
            comp_mul(&r, &cwc, &cnum);	/* r wc */
            comp_sub(&cnum, &cone, &ca);	/* a = 1 - r wc */
            comp_mul(&cnum, &cnum, &b4ac);	/* 1 - (r wc)^2 */
            comp_sub(&b4ac, &cone, &b4ac);
            b4ac.r *= 4.0;		  /* 4ac */
            b4ac.i *= 4.0;
            cb.r = -2.0 * cgam; /* b */
            cb.i = 0.0;
            comp_mul(&cb, &cb, &cnum);	/* b^2 */
            comp_sub(&b4ac, &cnum, &b4ac);	/* b^2 - 4 ac */
            err = comp_sqrt(&b4ac, &b4ac);
            if (err) return err;
            cb.r = -cb.r;		  /* -b */
            cb.i = -cb.i;
            ca.r *= 2.0;		  /* 2a */
            ca.i *= 2.0;
            comp_add(&b4ac, &cb, &cnum);	/* -b + sqrt( b^2 - 4ac) */
            err = comp_div(&ca, &cnum, &cnum);	/* ... /2a */
            if (err) return err;
            jt += 1;
            cmov(&cnum, &z[jt]);
            if (cnum.i != 0.0)
            {
               jt += 1;
               z[jt].r = cnum.r;
               z[jt].i = -cnum.i;
            }
            if ((r.i != 0.0) || (cnum.i == 0))
            {
               comp_sub(&b4ac, &cb, &cnum);	/* -b - sqrt( b^2 - 4ac) */
               err = comp_div(&ca, &cnum, &cnum);	/* ... /2a */
               if (err) return err;
               jt += 1;
               cmov(&cnum, &z[jt]);
               if (cnum.i != 0.0)
               {
                  jt += 1;
                  z[jt].r = cnum.r;
                  z[jt].i = -cnum.i;
               }
            }
         }
      }
      while (--nc > 0);
      if (icnt == 0)
      {
         zord = jt + 1;
         if (nz <= 0)
            break;
      }
      nc = nz;
   }
   
   /* Check zord */
   switch (type)
   {
      case BUTTER_LOWPASS:
      case BUTTER_HIGHPASS:
         zord_expected = order;
         break;
      case BUTTER_BANDPASS:
      case BUTTER_BANDSTOP:
         zord_expected = 2*order;
         break;
   }
   if (zord != zord_expected) return -2;

   lin[1].r = 1.0;
   lin[1].i = 0.0;
   /* Butterworth or Chebyshev */
   /* generate the remaining zeros */
   while (2 * zord - 1 > jt)
   {
      if (type != BUTTER_HIGHPASS) /* add zero at Nyquist freq */
      {
         jt += 1;
         z[jt].r = -1.0;	  /* zero at Nyquist frequency */
         z[jt].i = 0.0;
      }
      if ((type == BUTTER_BANDPASS) || (type == BUTTER_HIGHPASS)) /* add zero at 0Hz */
      {
         jt += 1;
         z[jt].r = 1.0;		  /* zero at 0 Hz */
         z[jt].i = 0.0;
      }
   }
   /* Expand the poles and zeros into numerator and
    * denominator polynomials */
   for (icnt = 0; icnt < 2; icnt++)
   {
      for (j=0; j<ARRSIZ; j++)
      {
         pp[j] = 0.0;
         y[j] = 0.0;
      }
      pp[0] = 1.0;
      for (j=0; j<zord; j++)
      {
         jj = j;
         if (icnt)
            jj += zord;
         a = z[jj].r;
         b = z[jj].i;
         for (i = 0; i <= j; i++)
         {
            jh = j - i;
            pp[jh + 1] = pp[jh + 1] - a * pp[jh] + b * y[jh];
            y[jh + 1] = y[jh + 1] - b * pp[jh] - a * y[jh];
         }
      }
      if (icnt == 0)
      {
         for (j = 0; j <= zord; j++)
            aa[j] = pp[j];
      }
   }
   /* Scale factors of the pole and zero polynomials */
   a = 1.0;
	switch (type)
	{
	case BUTTER_HIGHPASS:
		a = -1.0;
	case BUTTER_LOWPASS:
	case BUTTER_BANDSTOP:
		pn = 1.0;
		an = 1.0;
		for (j = 1; j <= zord; j++)
		{
			pn = a * pn + pp[j];
			an = a * an + aa[j];
		}
		break;
	case BUTTER_BANDPASS:
		gam = PI / 2.0 - asin(cgam);
		mh = zord / 2;
		pn = pp[mh];
		an = aa[mh];
		ai = 0.0;
		if (mh > ((zord / 4) * 2))
		{
			ai = 1.0;
			pn = 0.0;
			an = 0.0;
		}
		for (j = 1; j <= mh; j++)
		{
			a = gam * j - ai * PI / 2.0;
			cng = cos(a);
			jh = mh + j;
			jl = mh - j;
			pn = pn + cng * (pp[jh] + (1.0 - 2.0 * ai) * pp[jl]);
			an = an + cng * (aa[jh] + (1.0 - 2.0 * ai) * aa[jl]);
		}
	}
   
   gain = an / (pn * scale);
	if (pn == 0)
		gain = 1.0;
	for (j = 0; j <= zord; j++)
		pp[j] = gain * pp[j];
   
   /* Save to output */
	for (j=0; j<=zord; j++)
   {
      as[j] = aa[j];
      bs[j] = pp[j];
   }
   
	return 0;
}
