/**
 * ok but
 * have you ever
 * seen javascript
 * write *java*?
 * by extracting data from an HTML table
 * from a javadoc generated website?
 * 
 * there's a reason that this is really uncommon
 * and it's a good reason
 * 
 * anyways this file generates most of LazyWPITalonSRX.java
 * to use, go to https://www.ctr-electronics.com/downloads/api/java/html/classcom_1_1ctre_1_1phoenix_1_1motorcontrol_1_1can_1_1_w_p_i___talon_s_r_x.html
 * and paste this entire file into devtools and press enter
 * methods are copied to your clipboard
 */

// converts eg. (String moduleType, int moduleNumber, int channel) to (String, int, int)
const stripVarNames = signature => "(" + signature.slice(1, -1).split(", ").map(thing => thing.trim().split(" ")[0]).join(", ") + ")";

// converts eg. (String moduleType, int moduleNumber, int channel) to (moduleType, moduleNumber, channel)
const stripVarTypes = signature => "(" + signature.slice(1, -1).split(", ").map(thing => thing.trim().split(" ")[1]).join(", ") + ")";

const getReturnStrFromType = type => {
  switch (type){
        case "double":
        case "int":
        case "long":
            return "0";
        case "boolean":
            return "false";
        default:
            return "null";
  }
};

const rows =
// get every row of the table containing a method (ignore headings and spacers)
Array.from(document.querySelector(".memberdecls").children[0].querySelectorAll("[class^=memitem]")).
// convert that row into an array containing each column's value for this row
filter(row => row.querySelector(".memItemLeft").innerText.trim().length > 0).map(row => [row.querySelector(".memItemLeft").innerText.trim(), row.querySelector(".memItemRight").innerText.trim()]);

const dedupedRows = [];
for(let row of rows){
    const returnType = row[0];

    const spaceIdx = row[1].indexOf(" ");
    const methodName = row[1].slice(0, spaceIdx);
    const signature = stripVarNames(row[1].slice(spaceIdx + 1));
    if(!dedupedRows.some(otherRow => {
        const otherReturnType = otherRow[0];

        const otherSpaceIdx = otherRow[1].indexOf(" ");
        const otherMethodName = otherRow[1].slice(0, otherSpaceIdx);
        const otherSignature = stripVarNames(otherRow[1].slice(otherSpaceIdx + 1));

        return returnType === otherReturnType && methodName === otherMethodName && signature === otherSignature;
    }))
    dedupedRows.push(row);
}

const methods = [];
for(let row of dedupedRows){
    const split = row[0].split(" ");
    const type = split[split.length - 1];

    const spaceIdx = row[1].indexOf(" ");
    const methodName = row[1].slice(0, spaceIdx);
    const args = stripVarTypes(row[1].slice(spaceIdx + 1));
    methods.push(`
public ${row[0]} ${row[1]} {
  if(Robot.isReal()){
    ${type === "void" ? "" : "return"} thisTalon.${methodName}${args};
  }else{
    mLogger.verbose("${methodName}(${args.length > 2 ? '" + ' + args.slice(1, -1).split(", ").map(arg => "String.valueOf(" + arg + ")").join(' + ", " + ') + ' + "' : ""})");${type === "void" ? "" : `\n    return ${getReturnStrFromType(type)};`}
  }
}
`.trim());
}

copy(methods.join("\n"))