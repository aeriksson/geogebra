function mobile(){var bb='',kb='" for "gwt:onLoadErrorFn"',ib='" for "gwt:onPropertyErrorFn"',lb='#',wb='&',Wb='.cache.js',nb='/',Vb=':',cb='::',xb=';',R='<html><head><\/head><body><\/body><\/html>',fb='=',mb='?',hb='Bad handler "',W='DOMContentLoaded',M='DUMMY',Ub="GWT module 'mobile' may need to be (re)compiled",Ab='Unexpected exception in locale detection, using default: ',yb='_',zb='__gwt_Locale',Cb='android',Db='android_tablet',sb='base',qb='baseUrl',H='begin',Jb='blackberry',N='body',G='bootstrap',pb='clear.cache.gif',eb='content',Kb='desktop',ub='en',bc='end',Pb='file://',I='gwt.codesvr.mobile=',J='gwt.codesvr=',ac='gwt/clean/clean.css',jb='gwt:onLoadErrorFn',gb='gwt:onPropertyErrorFn',db='gwt:property',Z='head',$b='href',O='iframe',ob='img',Eb='ipad',Fb='ipad_retina',Gb='iphone',Hb='ipod',T='javascript',P='javascript:""',Xb='link',_b='loadExternalRefs',tb='locale',vb='locale=',$='meta',Bb='mgwt.os',K='mobile',Tb='mobile.devmode.js',rb='mobile.nocache.js',Lb='mobile.user.agent',ab='mobile::',Mb='mobilesafari',Y='moduleRequested',X='moduleStartup',_='name',Rb='no',Nb='not_mobile',Ob='phonegap.env',Q='position:absolute; width:0; height:0; border:none; left: -1000px; top: -1000px; !important',Yb='rel',Ib='retina',S='script',Sb='selectingPermutation',L='startup',Zb='stylesheet',V='undefined',U='var $wnd = window.parent;',Qb='yes';var t=window;var u=document;w(G,H);function v(){var a=t.location.search;return a.indexOf(I)!=-1||a.indexOf(J)!=-1}
function w(a,b){if(t.__gwtStatsEvent){t.__gwtStatsEvent({moduleName:K,sessionId:t.__gwtStatsSessionId,subSystem:L,evtGroup:a,millis:(new Date).getTime(),type:b})}}
mobile.__sendStats=w;mobile.__moduleName=K;mobile.__errFn=null;mobile.__moduleBase=M;mobile.__softPermutationId=0;mobile.__computePropValue=null;var x=function(){return false};var y=function(){return null};__propertyErrorFunction=null;function z(f){var g;function h(){j();return g}
function i(){j();return g.getElementsByTagName(N)[0]}
function j(){if(g){return}var a=u.createElement(O);a.src=P;a.id=K;a.style.cssText=Q;a.tabIndex=-1;u.body.appendChild(a);g=a.contentDocument;if(!g){g=a.contentWindow.document}g.open();g.write(R);g.close();var b=g.getElementsByTagName(N)[0];var c=g.createElement(S);c.language=T;var d=U;c.text=d;b.appendChild(c)}
function k(a){function b(){if(typeof u.readyState==V){return typeof u.body!=V&&u.body!=null}return /loaded|complete/.test(u.readyState)}
var c=false;if(b()){c=true;a()}var d;function e(){if(!c){c=true;a();if(u.removeEventListener){u.removeEventListener(W,e,false)}if(d){clearInterval(d)}}}
if(u.addEventListener){u.addEventListener(W,function(){e()},false)}var d=setInterval(function(){if(b()){e()}},50)}
function l(a){var b=i();var c=h().createElement(S);c.language=T;c.text=a;b.appendChild(c);b.removeChild(c)}
mobile.onScriptDownloaded=function(a){k(function(){l(a)})};w(X,Y);var m=u.createElement(S);m.src=f;u.getElementsByTagName(Z)[0].appendChild(m)}
function A(){var c={};var d;var e;var f=u.getElementsByTagName($);for(var g=0,h=f.length;g<h;++g){var i=f[g],j=i.getAttribute(_),k;if(j){j=j.replace(ab,bb);if(j.indexOf(cb)>=0){continue}if(j==db){k=i.getAttribute(eb);if(k){var l,m=k.indexOf(fb);if(m>=0){j=k.substring(0,m);l=k.substring(m+1)}else{j=k;l=bb}c[j]=l}}else if(j==gb){k=i.getAttribute(eb);if(k){try{d=eval(k)}catch(a){alert(hb+k+ib)}}}else if(j==jb){k=i.getAttribute(eb);if(k){try{e=eval(k)}catch(a){alert(hb+k+kb)}}}}}y=function(a){var b=c[a];return b==null?null:b};__propertyErrorFunction=d;mobile.__errFn=e}
function B(){function e(a){var b=a.lastIndexOf(lb);if(b==-1){b=a.length}var c=a.indexOf(mb);if(c==-1){c=a.length}var d=a.lastIndexOf(nb,Math.min(c,b));return d>=0?a.substring(0,d+1):bb}
function f(a){if(a.match(/^\w+:\/\//)){}else{var b=u.createElement(ob);b.src=a+pb;a=e(b.src)}return a}
function g(){var a=y(qb);if(a!=null){return a}return bb}
function h(){var a=u.getElementsByTagName(S);for(var b=0;b<a.length;++b){if(a[b].src.indexOf(rb)!=-1){return e(a[b].src)}}return bb}
function i(){var a=u.getElementsByTagName(sb);if(a.length>0){return a[a.length-1].href}return bb}
var j=g();if(j==bb){j=h()}if(j==bb){j=i()}if(j==bb){j=e(u.location.href)}j=f(j);return j}
function C(a){if(a.match(/^\//)){return a}if(a.match(/^[a-zA-Z]+:\/\//)){return a}return mobile.__moduleBase+a}
function D(){var m=[];var n;var o=[];var p=[];function q(a){var b=p[a](),c=o[a];if(b in c){return b}var d=[];for(var e in c){d[c[e]]=e}if(__propertyErrorFunc){__propertyErrorFunc(a,d,b)}throw null}
p[tb]=function(){var b=null;var c=ub;try{if(!b){var d=location.search;var e=d.indexOf(vb);if(e>=0){var f=d.substring(e+7);var g=d.indexOf(wb,e);if(g<0){g=d.length}b=d.substring(e+7,g)}}if(!b){var h=u.cookie;var i=h.indexOf(vb);if(i>=0){var g=h.indexOf(xb,i);if(g<0){g=h.length}b=h.substring(i+7,g)}}if(!b){var j=navigator.browserLanguage?navigator.browserLanguage:navigator.language;if(j){var k=j.split(/[-_]/);if(k.length>1){k[1]=k[1].toUpperCase()}b=k.join(yb)}}if(!b){b=t[zb]}if(b){c=b}while(b&&!x(tb,b)){var l=b.lastIndexOf(yb);if(l<0){b=null;break}b=b.substring(0,l)}}catch(a){alert(Ab+a)}t[zb]=c;return b||ub};o[tb]={ar:0,de:1,'default':2,en:3,es:4,fr:5,pt:6};p[Bb]=function(){{var b=function(){var a=window.navigator.userAgent.toLowerCase();if(a.indexOf(Cb)!=-1){if(a.indexOf(K)!=-1){return Cb}else{return Db}}if(a.indexOf(Eb)!=-1){if(window.devicePixelRatio>=2){return Fb}return Eb}if(a.indexOf(Gb)!=-1||a.indexOf(Hb)!=-1){if(window.devicePixelRatio>=2){return Ib}return Gb}if(a.indexOf(Jb)!=-1){return Jb}return Kb}();return b}};o[Bb]={android:0,android_tablet:1,blackberry:2,desktop:3,ipad:4,ipad_retina:5,iphone:6,retina:7};p[Lb]=function(){{var b=function(){var a=window.navigator.userAgent.toLowerCase();if(a.indexOf(Cb)!=-1){return Mb}if(a.indexOf(Gb)!=-1){return Mb}if(a.indexOf(Hb)!=-1){return Mb}if(a.indexOf(Eb)!=-1){return Mb}if(a.indexOf(Jb)!=-1){return Nb}return Nb}();return b}};o[Lb]={mobilesafari:0,not_mobile:1};p[Ob]=function(){{var a=window.navigator.userAgent.toLowerCase();if(a.indexOf(Cb)!=-1||(a.indexOf(Eb)!=-1||(a.indexOf(Hb)!=-1||(a.indexOf(Gb)!=-1||a.indexOf(Jb)!=-1)))){var b=document.location.href;if(b.indexOf(Pb)===0){return Qb}}return Rb}};o[Ob]={no:0,yes:1};x=function(a,b){return b in o[a]};mobile.__computePropValue=q;w(G,Sb);if(v()){return C(Tb)}var r;try{alert(Ub);return;var s=r.indexOf(Vb);if(s!=-1){n=r.substring(s+1);r=r.substring(0,s)}}catch(a){}mobile.__softPermutationId=n;return C(r+Wb)}
function E(){if(!t.__gwt_stylesLoaded){t.__gwt_stylesLoaded={}}function c(a){if(!__gwt_stylesLoaded[a]){var b=u.createElement(Xb);b.setAttribute(Yb,Zb);b.setAttribute($b,C(a));u.getElementsByTagName(Z)[0].appendChild(b);__gwt_stylesLoaded[a]=true}}
w(_b,H);c(ac);w(_b,bc)}
A();mobile.__moduleBase=B();var F=D();E();w(G,bc);z(F)}
mobile();