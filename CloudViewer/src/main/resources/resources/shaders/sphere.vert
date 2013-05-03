#pragma include includes/StdLib.frag
#define GAMMA
#pragma include includes/gamma.h

#define lambert_reflectance_VAL 0
#pragma brdf lambert

in vec3 normal;
in vec3 eye;

float saturate(float a)
{
    return min(1,max(0,a));
}

#define OneOnLN2_x6 8.656170 // == 1/ln(2) * 6   (6 is SpecularPower of 5 + 1)
vec3 FresnelSchlick(vec3 SpecularColor, vec3 E,vec3 H)
{
    return SpecularColor + (1.0f - SpecularColor) * exp2(-OneOnLN2_x6 * saturate(dot(E, H)));
}

void main()
{   
    vec3 N = normalize(normal);
    vec3 V = normalize(eye);
    vec3 L = normalize(vec3(0,1,1));
    vec3 H = normalize(L+V);

    float NdotL = dot(L, N);
    float NdotV = dot(V, N);

    //vec3 F = f_schlick_f0(L, V, N);
    //vec3 D = d_blinnphong(L, V, N);

//material properties
    vec3 SpecularColor = vec3(1  ,         0.765557  ,  0.336057);
    float gloss = 0.1;

//abgeleitetes material parameter
    float SpecularPower = exp2(10 * gloss + 1);

    vec3 spec = FresnelSchlick(SpecularColor, L, H) * ((SpecularPower + 2) / 8 ) * pow(saturate(dot(N, H)), SpecularPower) * NdotL;

    vec3 color = (lambert(L, V, N) + spec ) * max(0,NdotL);

    FragColor = toGamma(vec4(color, 1));
}