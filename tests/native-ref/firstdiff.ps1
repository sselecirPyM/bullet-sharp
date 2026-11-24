# Finds the first differing line between two trace dumps, comparing with the same
# numeric normalization as cmp.ps1 (exponent digit count is normalized away).
param([string]$a, [string]$b)

function Parse-nums([string]$line) {
    $nums = @()
    foreach ($m in [regex]::Matches($line, '[-+0-9.eE]+')) {
        $v = 0.0
        if ([double]::TryParse($m.Value, [System.Globalization.NumberStyles]::Float,
                [System.Globalization.CultureInfo]::InvariantCulture, [ref]$v)) { $nums += $v }
    }
    return $nums
}

$la = Get-Content $a
$lb = Get-Content $b
$n = [Math]::Min($la.Count, $lb.Count)
for ($i = 0; $i -lt $n; $i++) {
    $na = Parse-nums $la[$i]; $nb = Parse-nums $lb[$i]
    if ($na.Count -ne $nb.Count) {
        Write-Host "line $($i+1): field count differs"
        Write-Host "A: $($la[$i])"
        Write-Host "B: $($lb[$i])"
        if ($i -gt 0) { Write-Host "A prev: $($la[$i-1])"; Write-Host "B prev: $($lb[$i-1])" }
        exit 1
    }
    $diff = 0.0
    for ($j = 0; $j -lt $na.Count; $j++) { $diff = [Math]::Max($diff, [Math]::Abs($na[$j] - $nb[$j])) }
    if ($diff -gt 1e-300) {
        Write-Host "line $($i+1): maxabs diff $diff"
        Write-Host "A: $($la[$i])"
        Write-Host "B: $($lb[$i])"
        if ($i -gt 0) { Write-Host "A prev: $($la[$i-1])"; Write-Host "B prev: $($lb[$i-1])" }
        exit 1
    }
}
Write-Host "no differing line in first $n lines"
