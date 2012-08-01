<?php

// Loads a GGB file for testing.
// Example: http://URL?f=Girl_in_Mirror.ggb
// Parameters:
//  f: parameter file
//  s: show filelist (set 0 for off)
//  h: show hidden files in filelist (set 1 for on)
//  m: milestones directory
//  v: show version number (if possible, set 0 for off)
//  c: codebase

// Copyright (c) 2012 The GeoGebra Team <geogebra-dev@geogebra.org>

// Defaults, override them with parameters:
// (If you add more parameters, don't forget to add them into
// function passoptions() as well!)

// $MILESTONES="milestones/ggbFiles"; // Please don't use this any longer. It's too long.
$MILESTONES="test";

$DEFAULTFILE="Girl_in_Mirror.ggb";
//$CODEBASE="http://dev.geogebra.org/~build/trunk-geogebra/web/war/";

$SHOWFILELIST=TRUE;
$SHOWHIDDEN=FALSE;
$SHOWVERSION=TRUE;

if ($_GET['m']!="")
 $MILESTONES=$_GET['m'];
if ($_GET['s']=="0")
 $SHOWFILELIST=FALSE;
if ($_GET['h']=="1")
 $SHOWHIDDEN=TRUE;
if ($_GET['v']=="0")
 $VERSION=FALSE;
if ($_GET['c']=="0")
 $CODEBASE=$_GET['c'];

// Importing all milestones files into $filenames:
$dh=opendir($MILESTONES);
$filenames=array();
while($f=readdir($dh)) {
 $condition=TRUE;
 if ($SHOWHIDDEN)
  $condition=($f[0]!="_");
 if ($f[0]!="." && $condition)
  $filenames[]=$f;
 }
sort($filenames);

// Checking version
$FILE=$_GET['f'];
$TITLE=pretty_filename($FILE,".ggb");
if ($VERSION) {
 $VERSIONNUMBER=file_get_contents($CODEBASE."web/version.txt");
 if ($VERSIONNUMBER != "") {
  $TITLE="[$VERSIONNUMBER] $TITLE";
  }
 } 

?>
<!DOCTYPE html>
<html>
<head><?php echo style() ?>
<title><?php echo $TITLE ?></title>
<meta http-equiv="Content-Type" content="text/html; charset=utf-8" />
</head>
<body>
    <script type="text/javascript" language="javascript"
     src="<?php echo $CODEBASE ?>web/web.nocache.js"></script>
    <article class="geogebraweb"
    data-param-width="800"
    data-param-height="550"
    data-param-enableLabelDrags="false"
    data-param-enableShiftDragZoom="false"
    style="border: 1px solid black; display:inline-block;"
    data-param-ggbbase64=<?php

echo '"';
// Load the file:
$tempfile = in_array($FILE,$filenames) ? "$MILESTONES/$FILE" : "$MILESTONES/$DEFAULTFILE";
// Encode the file in base 64...
$handle = fopen($tempfile,'rb');
$file_content = fread($handle,filesize($tempfile));
fclose($handle);
//$encoded = chunk_split(base64_encode($file_content)); 
$encoded = base64_encode($file_content); 
// ... and print
echo $encoded;
echo '"'

?>></article><?php

// When asked, show the filelist:
if ($SHOWFILELIST) {
 echo "<div style=\"float:right;height:550px;overflow:auto\"><ul>";
 foreach ($filenames as $f) {
  
  echo "<li><a href=\"".$_SERVER['PHP_SELF']."?f=$f".passoptions()."\">".
   "<div style=\"".classify_filename($f)."\">".
   pretty_filename($f)."</div></a></li>";
  }
 echo "</ul></div></body></html>";
 }

function bool2int($bool) {
 if ($bool)
  return 1;
 return 0;
}

function passoptions() {
 global $MILESTONES, $CODEBASE, $SHOWFILELIST, $SHOWHIDDEN, $VERSION;
 return "&m=".$MILESTONES.
  "&c=".$CODEBASE.
  "&s=".bool2int($SHOWFILELIST).
  "&h=".bool2int($SHOWHIDDEN).
  "&v=".bool2int($VERSION);
 }

function pretty_filename($name) {
 return str_replace("_", " ", ltrim(basename($name, ".ggb"),"_"));
 }

function classify_filename($name) {
 global $FILE;
 if ($name[0]=="_")
  $style="color:#808080;";
 else
  $style="color:#200000;";
 if ($name==$FILE)
  $style.="background-color:yellow;";
 return $style;
 }

// Add here extra styles...
function style() {
 //return "<style>html,body { height: 100% }</style>";
 }

?>