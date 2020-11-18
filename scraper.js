// converts eg. (String moduleType, int moduleNumber, int channel) to (String, int, int)
stripVarNames = signature => "(" + signature.slice(1, -1).split(", ").map(thing => thing.trim().split(" ")[0]).join(", ") + ")";

// converts eg. (String moduleType, int moduleNumber, int channel) to (moduleType, moduleNumber, channel)
stripVarTypes = signature => "(" + signature.slice(1, -1).split(", ").map(thing => thing.trim().split(" ")[1]).join(", ") + ")";

rows=
// get every row of the table containing a method (ignore headings and spacers)
Array.from(document.querySelector(".memberdecls").children[0].querySelectorAll("[class^=memitem]")).
// convert that row into an array containing each column's value for this row
filter(row => row.querySelector(".memItemLeft").innerText.trim().length > 0).map(row => [row.querySelector(".memItemLeft").innerText.trim(), row.querySelector(".memItemRight").innerText.trim()]);

dedupedRows = [];
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
methods = [];
for(let row of dedupedRows){
    const split = row[0].split(" ");
    const type = split[split.length - 1];

    const spaceIdx = row[1].indexOf(" ");
    const methodName = row[1].slice(0, spaceIdx);
    const args = stripVarTypes(row[1].slice(spaceIdx + 1));
    methods.push(`public ${row[0]} ${row[1]} {${type === "void" ? "" : "return"} thisTalon.${methodName}${args};}`);
}
methods.join("\n")